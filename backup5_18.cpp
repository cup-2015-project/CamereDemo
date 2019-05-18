// demo3.cpp : 定义控制台应用程序的入口点。
//双线程播放监控摄像头，线程一调用摄像头读取帧到Mat队列，线程二从Mat队列读取帧并检测运动目标

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
#include <string>

#include "matList.h"
#include "histogram.h"

#include "Windows.h"
#include "HCNetSDK.h"
#include "plaympeg4.h"


using namespace std;
using namespace cv;




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

int FrameWidth;
int FrameHeight;
int Situation = 0;
int Turning = 0;
mutex mu1;
condition_variable cond1;

///////////////////////////////////
////////系列回调函数

//--------------------------------------------
int iPicNum = 0;//Set channel NO.
LONG nPort = -1;
HWND hWnd = NULL;



// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


//解码回调 视频为YUV数据(YV12)，音频为PCM数据
void CALLBACK DecCBFun(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long nReserved2)
{
	long lFrameType = pFrameInfo->nType;

	FrameHeight = pFrameInfo->nHeight;//全局变量记录帧长宽
	FrameWidth = pFrameInfo->nWidth;

	if (lFrameType == T_YV12)
	{
		Mat pImg_YUV(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, pBuf);
		
		//直接将YV12格式的buffer转成Mat，写入全局链队列
		mat_list.pushNewFrame(pImg_YUV);
	}
}


///实时流回调
void CALLBACK fRealDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *pUser)
{
	DWORD dRet;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD:    //系统头
		if (!PlayM4_GetPort(&nPort)) //获取播放库未使用的通道号
		{
			break;
		}
		if (dwBufSize > 0)
		{
			if (!PlayM4_OpenStream(nPort, pBuffer, dwBufSize, 1024 * 1024))
			{
				dRet = PlayM4_GetLastError(nPort);
				break;
			}
			//设置解码回调函数 只解码不显示
			if (!PlayM4_SetDecCallBack(nPort, DecCBFun))
			{
				dRet = PlayM4_GetLastError(nPort);
				break;
			}

			//设置解码回调函数 解码且显示
			//if (!PlayM4_SetDecCallBackEx(nPort,DecCBFun,NULL,NULL))
			//{
			//	dRet=PlayM4_GetLastError(nPort);
			//	break;
			//}

			//打开视频解码
			if (!PlayM4_Play(nPort, hWnd))
			{
				dRet = PlayM4_GetLastError(nPort);
				break;
			}

			//打开音频解码, 需要码流是复合流
			//if (!PlayM4_PlaySound(nPort))
			//{
			//	dRet = PlayM4_GetLastError(nPort);
			//	break;
			//}
		}
		break;

	case NET_DVR_STREAMDATA:   //码流数据
		if (dwBufSize > 0 && nPort != -1)
		{
			BOOL inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
			while (!inData)
			{
				Sleep(10);
				inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
				OutputDebugString(L"PlayM4_InputData failed \n");
			}
		}
		break;
	}
}

void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
	char tempbuf[256] = { 0 };
	switch (dwType)
	{
	case EXCEPTION_RECONNECT:    //预览时重连
		printf("----------reconnect--------%d\n", time(NULL));
		break;
	default:
		break;
	}
}



void turnCamera(LONG lRealPlayHandle)
{
	int situation = 0;
	int speed = 7;
	/********
	A
	5 | 1 | 6
	---------
	< 3 | 0 | 4 >
	---------
	7 | 2 | 8
	V
	*********/
	while (1)
	{
		std::unique_lock<std::mutex> locker1(mu1);//lock

		if (Situation == -1)
		{
			Situation = 0;
			locker1.unlock();//unlock
			break;
		}
		else if (Situation == -2)
		{
			Situation = 0;
			locker1.unlock();//unlock
			Sleep(1000);
			continue;
		}
		situation = Situation;
		Situation = 0;
		locker1.unlock();//unlock
		switch (situation)
		{
		case 1:
			//move
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_DOWN, 0, speed);
			//cout << "start A" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_DOWN, 1, speed);
			Turning = 0;
			//cout << "stop A" << endl;
			//stop
			break;
		case 2:
			//move
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 0, speed);
			//cout << "start V" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 1, speed);
			Turning = 0;
			//cout << "stop V" << endl;
			//stop
			break;
		case 3:
			//move  PAN_RIGHT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 0, speed);
			//cout << "start <" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 1, speed);
			Turning = 0;
			//cout << "stop <" << endl;
			//stop
			break;
		case 4:
			//move  PAN_LEFT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_LEFT, 0, speed);
			//cout << "start >" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_LEFT, 1, speed);
			Turning = 0;
			//cout << "stop >" << endl;
			//stop
			break;
		case 5:
			//move  DOWN_RIGHT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_RIGHT, 0, speed);
			//cout << "start 5" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_RIGHT, 1, speed);
			Turning = 0;
			//cout << "stop 5" << endl;
			//stop
			break;
		case 6:
			//move  DOWN_LEFT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_LEFT, 0, speed);
			//cout << "start 6" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_LEFT, 1, speed);
			Turning = 0;
			//cout << "stop 6" << endl;
			//stop
			break;
		case 7:
			//move  UP_RIGHT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 0, speed);
			//cout << "start 7" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 1, speed);
			Turning = 0;
			//cout << "stop 7" << endl;
			//stop
			break;
		case 8:
			//move  UP_LEFT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_LEFT, 0, speed);
			//cout << "start 8" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_LEFT, 1, speed);
			Turning = 0;
			//cout << "stop 8" << endl;
			//stop
			break;
		default:
			break;
		}
		Sleep(1000);
	}
}



///////////////////////////////////
void readFrame()
{

	//---------------------------------------
	// 初始化
	NET_DVR_Init();
	//设置连接时间与重连时间
	NET_DVR_SetConnectTime(2000, 1);
	NET_DVR_SetReconnect(10000, true);

	//---------------------------------------
	// 获取控制台窗口句柄
	//HMODULE hKernel32 = GetModuleHandle((LPCWSTR)"kernel32");
	//GetConsoleWindow = (PROCGETCONSOLEWINDOW)GetProcAddress(hKernel32,"GetConsoleWindow");

	//---------------------------------------
	// 注册设备
	LONG lUserID;
	NET_DVR_DEVICEINFO_V30 struDeviceInfo;
	lUserID = NET_DVR_Login_V30("192.168.1.64", 8000, "admin", "a1234567", &struDeviceInfo);
	if (lUserID < 0)
	{
		printf("Login error, %d\n", NET_DVR_GetLastError());
		NET_DVR_Cleanup();
		return;
	}

	//---------------------------------------
	//设置异常消息回调函数
	NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);


	//cvNamedWindow("IPCamera");
	//---------------------------------------
	//启动预览并设置回调数据流 
	NET_DVR_CLIENTINFO ClientInfo;
	ClientInfo.lChannel = 1;        //Channel number 设备通道号
	ClientInfo.hPlayWnd = NULL;     //窗口为空，设备SDK不解码只取流
	ClientInfo.lLinkMode = 0;       //Main Stream
	ClientInfo.sMultiCastIP = NULL;

	LONG lRealPlayHandle;
	lRealPlayHandle = NET_DVR_RealPlay_V30(lUserID, &ClientInfo, fRealDataCallBack, NULL, TRUE);
	if (lRealPlayHandle<0)
	{
		printf("NET_DVR_RealPlay_V30 failed! Error number: %d\n", NET_DVR_GetLastError());
		return;
	}



	turnCamera(lRealPlayHandle);
	cout << "turn end" << endl;
	Sleep(-1);
	//---------------------------------------
	//关闭预览
	if (!NET_DVR_StopRealPlay(lRealPlayHandle))
	{
		printf("NET_DVR_StopRealPlay error! Error number: %d\n", NET_DVR_GetLastError());
		return;
	}

	//注销用户
	NET_DVR_Logout(lUserID);
	NET_DVR_Cleanup();

	return;
}


Mat read_Frame()
{
	Mat temp_frame = mat_list.getNewFrame();
	//取出YV12格式的Mat进行转换、显示
	Mat pImg(FrameHeight, FrameWidth, CV_8UC3);
	//Mat pImg_YUV(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, pBuf);
	Mat pImg_YCrCb(FrameHeight, FrameWidth, CV_8UC3);
	cvtColor(temp_frame, pImg, CV_YUV2BGR_YV12);
	cvtColor(pImg, pImg_YCrCb, CV_BGR2YCrCb);
	return pImg;
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
	Ptr<Tracker> * tracker = new Ptr<Tracker>;

#if (CV_MINOR_VERSION < 3)
	{
		*tracker = Tracker::create(trackerType);
	}
#else
	{
		*tracker = TrackerMOSSE::create();
	}
#endif

	int count_frame = 0;
	// Define initial boundibg box 
	Rect2d bbox;
	Rect2d bboxPre;

	Point tl;
	Point br ;
	Point center_bbox ;
	Rect2d image0;
	Point tl0;
	Point br0 ;
	Point center_0 ;

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
			bbox = selectROI(frame, false);

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
			(*tracker)->init(frame, bbox);

		}
		//tracing
		else
		{
			// Start timer
			timer = (double)getTickCount();
			
			switch (state)
			{
			case 1:

				

				// Update the tracking result
				ok = (*tracker)->update(frame, bbox);

			

				tl = bbox.tl();
				br = bbox.br();
				center_bbox = (br - tl) / 2 + tl;

				image0 = getWindowImageRect("Tracking");
				tl0 = image0.tl();
				br0 = image0.br();
				center_0 = (br0 - tl0) / 2;


				if (ok)
				{
					// Tracking success : Draw the tracked object
					rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);

					/********
					A
					5 | 1 | 6
					---------
					< 3 | 0 | 4 >
					---------
					7 | 2 | 8
					V
					*********/
					bool s1 = false;
					bool s2 = false;
					bool s3 = false;
					bool s4 = false;
					int temp_s = 0;

					if (center_bbox.x - center_0.x > 30)
					{
						s3 = true;
						putText(frame, "<", Point(70, 140), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					}
					else if (center_bbox.x - center_0.x < -30)
					{
						s4 = true;
						putText(frame, ">", Point(130, 140), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					}
					if (center_bbox.y - center_0.y > 30)
					{
						s1 = true;
						putText(frame, "A", Point(100, 110), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					}
					else if (center_bbox.y - center_0.y < -30)
					{
						s2 = true;
						putText(frame, "V", Point(100, 170), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					}

					if (s1&&s3)
					{
						temp_s = 5;
					}
					else if (s1&&s4)
					{
						temp_s = 6;
					}
					else if (s2&&s3)
					{
						temp_s = 7;
					}
					else if (s2&&s4)
					{
						temp_s = 8;
					}
					else if (s1)
					{
						temp_s = 1;
					}
					else if (s2)
					{
						temp_s = 2;
					}
					else if (s3)
					{
						temp_s = 3;
					}
					else if (s4)
					{
						temp_s = 4;
					}
					std::unique_lock<std::mutex> locker1(mu1);//lock
					Situation = temp_s;

					locker1.unlock();//unlock
				}
				else
				{
					// Tracking failure detected.
					putText(frame, "1Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
					bboxPre = Rect2d(bbox);

					state = 2;

					std::unique_lock<std::mutex> locker1(mu1);//lock
					Situation = -2;
					locker1.unlock();//unlock

				}
				break;

			case 2:
				if (count_frame > 3 && Turning == 0)
				{
					
					Mat result_frame = detect3frames(frameNow, framePre, framePrePre);
					Rect2d rect = printContours(result_frame);

					rectangle(frame, rect, Scalar(0, 255, 0), 2, 1);
					
					if (!(rect.area() >= 50 && rect.area() <= bboxPre.size().area() * 12) )
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
				putText(frame, "2Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);

				break;

			case 3:

				delete tracker;

				tracker = new Ptr<Tracker>;
#if (CV_MINOR_VERSION < 3)
			{
				*tracker = Tracker::create(trackerType);
			}
#else
			{
				*tracker = TrackerMOSSE::create();
			}
#endif

				(*tracker)->init(frame, bbox);
				rectangle(frame, bbox, Scalar(0, 255, 0), 2, 1);
				state = 1;

				break;

			default:
				break;
			}

			// Display tracker type on frame
			//putText(frame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

			// Calculate Frames per second (FPS)
			fps = getTickFrequency() / ((double)getTickCount() - timer);


			// Display FPS on frame
			putText(frame, "FPS : " + SSTR(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);


			// Display frame.
			imshow("Tracking", frame);

			// Exit if ESC pressed.
			int k = waitKey(1) & 0xFF;
			if (k == 27)
			{
				break;
			}
		}

		Mat frame16u;
		frame.convertTo(frame16u, CV_16U);
		string fileName = "";
		fileName += "D:\\images_test\\src_"+SSTR(int(count_frame))+".jpg";
		imwrite(fileName, frame);
		//imshow("temp" , frame);
		//waitKey(1);
	}
	cv::destroyAllWindows();
	delete tracker;
	return;
}



void showHist() 
{

		//读取帧
		Mat src = mat_list.getNewFrame();
		Mat hsv;

		cvtColor(src, hsv, CV_BGR2HSV);
		imshow("src", src);
		imshow("hsv", hsv);
		CalcHistogramHS h;
		h.getHistogram(hsv);
		h.getHistogramImage(hsv);
		
		waitKey(0);
		return  ;
	
}

int main()
{

//	thread reader(write_Frame);
//	//Sleep(1);
//	thread tracker(showHist);
//

	thread reader(readFrame);
	//Sleep(1);
	thread tracker(track);

	reader.join();
	tracker.join();


	system("pause");
	return 0;
}




