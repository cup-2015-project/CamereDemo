//三帧差帧法、跟踪检测结合仍需调试
//需要matList.h

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

#include "matList.h"


using namespace std;
using namespace cv;


#define MAX_SIZE 70
#define MIN_BUFFUR_SIZE 5
#define TOTAL 500

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


//CRITICAL_SECTION g_cs_frameList;
//std::list<Mat> g_frameList;
//list<int> testList;
//int i = -1;
//mutex mu;
//condition_variable cond;
extern matList mat_list;


void write_Frame()
{
	Mat temp_frame;

	//int i = 0;

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
		
		mat_list.pushNewFrame(temp_frame);

		//if (i > 50)
		//	break;
		//++i;
	}

	cap.release();
	return;
}

Mat read_Frame()
{
	return mat_list.getNewFrame();
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
void detect(Rect2d &rect_)
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
			//rectangle(temp_frame, rect, Scalar(0, 255, 0), 2, 8); //绘制最大的外接矩形
			//rectangle(result_frame, rect, Scalar(0, 255, 0), 2, 8); //绘制最大的外接矩形

			if(rect.area() >= 50 && rect.area() <= temp_frame.size().area() / 3)
			{
				rect_ = rect;
				break;
			}
			//imshow("result", result_frame);

		}
		//imshow("temp", temp_frame);
		//waitKey(1);
	}
	//destroyAllWindows();
	return;
}

void track()
{
	Mat frame, frameNow, framePre, framePrePre;
	//Mat h;
	int state = 0;
	float fps = 0;
	double timer = 0;
	bool ok = false;
	bool flag2 = false;
	string trackerType = "MOSSE";
	Ptr<Tracker> tracker;

#if (CV_MINOR_VERSION < 3)
	{
		tracker = Tracker::create(trackerType);
	}
#else
	{
		tracker = TrackerMOSSE::create();
	}
#endif

	int count_frame = 0;
	// Define initial boundibg box 
	Rect2d bbox;
	Rect2d bboxPre;


	while (1)
	{
		//read frame
		frame = read_Frame();

		//缓存
		framePrePre = framePre.clone();
		framePre = frameNow.clone();
		frameNow = frame.clone();

		//处理过程
		//select ROI
		count_frame++;
		if (1 == count_frame)
		{
			// Uncomment the line below to select a different bounding box 
			// = selectROI(frame, false);

			if (bbox == Rect2d(0, 0, 0, 0))
			{
				Rect2d temp;
				detect(temp);
				double x = temp.x;
				double y = temp.y;
				double w = temp.width;
				double h = temp.height;
				bbox = Rect2d(x + w / 2 - 50, y + h / 2 - 50, 100, 100);
				//bbox = temp;
			}

			state = 1;

			// Display bounding box. 
			rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
			imshow("Tracking", frame);
			tracker->init(frame, bbox);

		}
		//tracing
		else
		{
			switch (state)
			{
			case 1:

				// Start timer
				timer = (double)getTickCount();

				// Update the tracking result
				ok = tracker->update(frame, bbox);

				// Calculate Frames per second (FPS)
				fps = getTickFrequency() / ((double)getTickCount() - timer);

				// Display FPS on frame
				//putText(frame, "1FPS : " + SSTR(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

				if (ok)
				{
					// Tracking success : Draw the tracked object
					rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);

				}
				else
				{
					// Tracking failure detected.
					//putText(frame, "1Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
					bboxPre = Rect2d(bbox);

					state = 2;
				}
				break;

			case 2:
				if (count_frame > 3)
				{
					
					Mat result_frame = detect3frames(frameNow, framePre, framePrePre);
					Rect2d rect = printContours(result_frame);

					if (!(rect.area() >= 50 && rect.area() <= frameNow.size().area() / 3) )
					{
						break;
					}

					cout << count_frame << endl;
					Point2d center = Point2d((rect.x + rect.width / 2), (rect.y + rect.height / 2));
					Size size_bbox = Size(bboxPre.size());

					Rect2d test_bbox = Rect2d(Point(center.x - 0.5*size_bbox.width, center.y - 0.75*size_bbox.height), size_bbox);
					
					bbox = Rect2d(test_bbox);
					state = 3;
					//bool ok = tracker->update(frame, test_bbox);

					//if (ok)
					//{
					//	bbox = Rect2d(test_bbox);
					//	state = 1;
					//}
					//else
					//{
					//	framePre = frame.clone();
					//}
					
				}
				rectangle(frame, bboxPre, Scalar(0, 0, 255), 2, 1);
				// Tracking failure detected.
				//putText(frame, "2Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);

				break;

			case 3:


				tracker->update(frame, bbox);
				rectangle(frame, bbox, Scalar(0, 255, 0), 2, 1);
				state = 1;


				//// Start timer
				//timer = (double)getTickCount();

				//// Update the tracking result
				//ok = tracker->update(frame, bbox);

				//// Calculate Frames per second (FPS)
				//fps = getTickFrequency() / ((double)getTickCount() - timer);

				//// Display FPS on frame
				//putText(frame, "3FPS : " + SSTR(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

				//if (ok)
				//{
				//	// Tracking success : Draw the tracked object
				//	rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);

				//	state = 1;
				//}
				//else
				//{
				//	// Tracking failure detected.
				//	putText(frame, "3Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
				//	
				//	state = 2;
				//}
				break;

			default:
				break;
			}

			// Display tracker type on frame
			//putText(frame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);



			// Display frame.
			imshow("Tracking", frame);

			// Exit if ESC pressed.
			int k = waitKey(1) & 0xFF;
			if (k == 27)
			{
				break;
			}
		}

		//imshow("temp" , frame);
		//waitKey(1);
	}
	cv::destroyAllWindows();
	return;
}


int main()
{
	thread reader(write_Frame);
	//Sleep(1);
	thread tracker(track);


	reader.join();
	tracker.join();

	//cout << mat_list.g_frameList.size() <<endl;

	//thread reader1(write_Frame);
	//reader1.join();

	//cout << mat_list.g_frameList.size() << endl;

	//thread reader2(write_Frame);
	//reader2.join();

	//cout << mat_list.g_frameList.size() << endl;
	//cout << testList.max_size() << endl;
	system("pause");
	return 0;
}
