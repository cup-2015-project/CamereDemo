// demo3.cpp : 定义控制台应用程序的入口点。
//双线程播放笔记本摄像头，线程一调用摄像头读取帧到Mat队列，线程二从Mat队列读取帧并检测运动目标

#include "stdafx.h"
#include "Windows.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>



using namespace std;
using namespace cv;


#define MAX_SIZE 70
#define MIN_BUFFUR_SIZE 5
#define TOTAL 500

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


//CRITICAL_SECTION g_cs_frameList;
std::list<Mat> g_frameList;
list<int> testList;
int i = -1;
mutex mu;
condition_variable cond;

void write_Frame()
{
	Mat temp_frame;

	VideoCapture cap(0);
	if (!cap.isOpened())
		return;

	while (1)
	{
		cap >> temp_frame;
		if (temp_frame.empty())
		{
			break;
		}
		std::unique_lock<std::mutex> locker(mu);//lock
		if (MAX_SIZE == g_frameList.size())
			g_frameList.pop_front();
		g_frameList.push_back(temp_frame);
		++i;
		//if (i > TOTAL)
		//{
		//	g_frameList.clear();
		//	locker.unlock();//unlock
		//	break;
		//}

		//locker.unlock();//unlock
		//cond.notify_one();
		if (MIN_BUFFUR_SIZE < g_frameList.size())
			cond.notify_one();//unlock
		else
			locker.unlock();//unlock

		Sleep(10);
	}

	cap.release();
	return;
}

Mat read_Frame()
{
	Mat temp_frame;

	std::unique_lock<std::mutex> locker(mu);//lock
	while (g_frameList.empty())
	{
		cond.wait(locker);
	}
	//if (i > TOTAL - 1)//确定结束条件，处理帧停止后还需收集帧，否则无法唤醒处理程序
	//{
	//	locker.unlock();//unlock
	//	destroyAllWindows();
	//	return;
	//}
	list<Mat>::iterator it;
	it = g_frameList.end();
	it--;
	temp_frame = *it;
	//cout << "size:" << g_frameList.size() << endl;
	//it--;
	//it--;
	//temp_frame0 = *it;
	locker.unlock();//unlock

	return temp_frame;
}


/*****************
改进三帧差帧法
输入：当前帧及前两帧
输出：运动目标在当前帧的二值图像
***************/
Mat detect3frames(Mat &frame, Mat &framePre, Mat &framePrePre)
{
	Mat diffImg1, diffImg2, grayDiffImg1, grayDiffImg2;
	Mat biImg12, biImg23;
	Mat biImg1, biImg2, biImg3;
	Mat result;

	absdiff(framePrePre, framePre, diffImg1);
	absdiff(framePre, frame, diffImg2);

	cvtColor(diffImg1, grayDiffImg1, CV_BGR2GRAY);
	cvtColor(diffImg2, grayDiffImg2, CV_BGR2GRAY);

	threshold(grayDiffImg1, biImg12, 15, 255, THRESH_BINARY);
	threshold(grayDiffImg2, biImg23, 15, 255, THRESH_BINARY);

	bitwise_and(biImg12, biImg23, biImg2);

	bitwise_xor(biImg2, biImg23, biImg3);

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat element2 = getStructuringElement(MORPH_RECT, Size(3, 3));

	morphologyEx(biImg3,result, MORPH_OPEN,element);
	morphologyEx(result, result, MORPH_CLOSE, element);
	morphologyEx(result, result, MORPH_CLOSE, element);
	//morphologyEx(result, result, MORPH_OPEN, element);
	//morphologyEx(result, result, MORPH_OPEN, element);

	return result;
}

/*******
输入：形态学处理后的二值图像
输出：1值像素的外接矩形
******/
Rect2d printContours(Mat &frame)
{
	vector<vector<Point>> contours;
	vector<Vec4i> hierarcy;
	findContours(frame, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE); //查找轮廓
	vector<Rect> boundRect(contours.size()); //定义外接矩形集合
	//drawContours(img2, contours, -1, Scalar(0, 0, 255), 1, 8);  //绘制轮廓

	int x0 = 0, y0 = 0, w0 = 0, h0 = 0;
	Rect2d rect(0, 0, 0, 0);
	for (int i = 0; i<contours.size(); i++)
	{
		boundRect[i] = boundingRect((Mat)contours[i]); //查找每个轮廓的外接矩形
		x0 = boundRect[i].x;  //获得第i个外接矩形的左上角的x坐标
		y0 = boundRect[i].y; //获得第i个外接矩形的左上角的y坐标
		w0 = boundRect[i].width; //获得第i个外接矩形的宽度
		h0 = boundRect[i].height; //获得第i个外接矩形的高度
		rect = rect | (Rect2d(x0, y0, w0, h0));
		//rectangle(temp_frame, Point(x0, y0), Point(x0 + w0, y0 + h0), Scalar(0, 255, 0), 2, 8); //绘制第i个外接矩形
	}
	//rectangle(temp_frame, rect, Scalar(0, 0, 255), 2, 8); //绘制最大的外接矩形

	//imshow("temp", temp_frame);
	//imshow("test", result_frame);

	return rect;
}


//Rect2d &rect_
void detect()
{
	Mat temp_frame,temp_frame0,temp_frame1;
	Mat result_frame;
	int index = 0;
	while (1)
	{
		
		temp_frame = read_Frame();

		++index;
		//cout << index << endl;
		if (index == 1)
		{
			temp_frame0 = temp_frame.clone();
		}
		else if(index == 2)
		{
			temp_frame1 = temp_frame0.clone();
			temp_frame0 = temp_frame.clone();
		}
		else
		{
			Rect2d rect(0, 0, 0, 0);
			
			result_frame = detect3frames(temp_frame,temp_frame0,temp_frame1);
			//cout << "frame size: "<<result_frame.size << endl;
			temp_frame1 = temp_frame0.clone();
			temp_frame0 = temp_frame.clone();
			

			rect = printContours(result_frame);
			rectangle(temp_frame, rect, Scalar(0, 255, 0), 2, 8); //绘制最大的外接矩形
			rectangle(result_frame, rect, Scalar(0, 255, 0), 2, 8); //绘制最大的外接矩形


			imshow("result", result_frame);

			//if (rect == Rect2d(0, 0, 0, 0))
			//	continue;
			////rect_ = rect;
			//break;
			////waitKey(1);
		}
		imshow("temp", temp_frame);
		

		waitKey(1);
	}
	//destroyAllWindows();
	return;
}




int main()
{
	thread reader(write_Frame);
	//Sleep(1);
	thread tracker(detect);


	reader.join();
	tracker.join();

	//cout << g_frameList.max_size() <<endl;
	//cout << testList.max_size() << endl;
	system("pause");
	return 0;
}
