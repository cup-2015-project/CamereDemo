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
#include <cmath>

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

extern matList mat_list;

//测试用函数，调用笔记本摄像头得到的帧写入队列
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

///解码回调 视频为YUV数据(YV12)，音频为PCM数据
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

///异常捕获回调函数
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

//控制摄像头旋转，需要播放句柄传入
void turnCamera(LONG lRealPlayHandle)
{
	int situation = 0;
	int speed = 7;
	int time_wait = 800;
	int time_move = 50;

	/**************
	*      A
	*  5 | 1 | 6
	*  ---------
	*< 3 | 0 | 4 >
	*  ---------
	*  7 | 2 | 8
	*      V
	**************/
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
			Sleep(time_move);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_DOWN, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop A" << endl;
			//stop
			break;
		case 2:
			//move
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 0, speed);
			//cout << "start V" << endl;
			Sleep(time_move);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop V" << endl;
			//stop
			break;
		case 3:
			//move  PAN_RIGHT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 0, speed);
			//cout << "start <" << endl;
			Sleep(time_move);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop <" << endl;
			//stop
			break;
		case 4:
			//move  PAN_LEFT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_LEFT, 0, speed);
			//cout << "start >" << endl;
			Sleep(time_move);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_LEFT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop >" << endl;
			//stop
			break;
		case 5:
			//move  DOWN_RIGHT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_RIGHT, 0, speed);
			//cout << "start 5" << endl;
			Sleep(time_move);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_RIGHT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop 5" << endl;
			//stop
			break;
		case 6:
			//move  DOWN_LEFT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_LEFT, 0, speed);
			//cout << "start 6" << endl;
			Sleep(time_move);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_LEFT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop 6" << endl;
			//stop
			break;
		case 7:
			//move  UP_RIGHT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 0, speed);
			//cout << "start 7" << endl;
			Sleep(time_move);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop 7" << endl;
			//stop
			break;
		case 8:
			//move  UP_LEFT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_LEFT, 0, speed);
			//cout << "start 8" << endl;
			Sleep(time_move);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_LEFT, 1, speed);
			Sleep(time_wait);
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
//控制摄像头旋转，需要播放句柄传入
void turnCamera2(LONG lRealPlayHandle)
{
	int situation = 0;
	int speed = 7;
	int time_wait = 800;
	int time_move34 = 370;
	int time_move12 = 250;
	int time_move5678 = 250;

	/**************
	*      A
	*  5 | 1 | 6
	*  ---------
	*< 3 | 0 | 4 >
	*  ---------
	*  7 | 2 | 8
	*      V
	**************/
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
			Sleep(time_move12);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_DOWN, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop A" << endl;
			//stop
			break;
		case 2:
			//move
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 0, speed);
			//cout << "start V" << endl;
			Sleep(time_move12);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop V" << endl;
			//stop
			break;
		case 3:
			//move  PAN_RIGHT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 0, speed);
			//cout << "start <" << endl;
			Sleep(time_move34);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop <" << endl;
			//stop
			break;
		case 4:
			//move  PAN_LEFT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_LEFT, 0, speed);
			//cout << "start >" << endl;
			Sleep(time_move34);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_LEFT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop >" << endl;
			//stop
			break;
		case 5:
			//move  DOWN_RIGHT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_RIGHT, 0, speed);
			//cout << "start 5" << endl;
			Sleep(time_move5678);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_RIGHT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop 5" << endl;
			//stop
			break;
		case 6:
			//move  DOWN_LEFT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_LEFT, 0, speed);
			//cout << "start 6" << endl;
			Sleep(time_move5678);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_LEFT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop 6" << endl;
			//stop
			break;
		case 7:
			//move  UP_RIGHT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 0, speed);
			//cout << "start 7" << endl;
			Sleep(time_move5678);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 1, speed);
			Sleep(time_wait);
			Turning = 0;
			//cout << "stop 7" << endl;
			//stop
			break;
		case 8:
			//move  UP_LEFT
			Turning = 1;
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_LEFT, 0, speed);
			//cout << "start 8" << endl;
			Sleep(time_move5678);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_LEFT, 1, speed);
			Sleep(time_wait);
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
//读帧线程，连接监控摄像头，启动新的线程将摄像头得到的帧写入队列
//同时调用控制摄像头转动的函数
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

	//Sleep(5000);
	//NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 0, 7);
	//cout << "start <" << endl;
	//Sleep(370);
	//NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 1, 7);
	
	//NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 0, 7);
	////cout << "start V" << endl;
	//Sleep(250);
	//NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 1, 7);
	//NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 0, 7);
	////cout << "start V" << endl;
	//Sleep(250);
	//NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 1, 7);

	
	turnCamera2(lRealPlayHandle);

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

//将队列中的帧读出并转换格式（YV12->BGR）
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

	threshold(grayDiffImg1, biImg12, 35, 255, THRESH_BINARY);
	threshold(grayDiffImg2, biImg23, 35, 255, THRESH_BINARY);

	bitwise_and(biImg12, biImg23, biImg2);

	bitwise_xor(biImg2, biImg23, biImg3);

	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	Mat element2 = getStructuringElement(MORPH_RECT, Size(15, 15));

	result = biImg3;
	//形态学滤波
	morphologyEx(result, result, MORPH_CLOSE, element2);
	morphologyEx(result, result, MORPH_CLOSE, element2);
	morphologyEx(result, result, MORPH_CLOSE, element2);
	morphologyEx(result, result, MORPH_CLOSE, element2);
	morphologyEx(result, result, MORPH_CLOSE, element2);
	morphologyEx(result, result, MORPH_OPEN, element);
	//morphologyEx(result, result, MORPH_CLOSE, element2);
	//dilate(result, result, element2);
	//morphologyEx(result, result, MORPH_CLOSE, element2);
	//morphologyEx(result, result, MORPH_OPEN, element);

	return result;
}


/*******
输入：形态学处理后的二值图像；存储检测到轮廓外接矩形的集合(vector<Rect>)
{)；
输出：1值像素的外接矩形集合数目
******/
int printContours(Mat &frame, vector<Rect> &boundRect)
{
	vector<vector<Point>> contours;
	vector<Vec4i> hierarcy;
	findContours(frame, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE); //查找轮廓
	int contours_size = contours.size();
	boundRect = vector<Rect>(contours_size); //定义外接矩形集合

	int x0 = 0, y0 = 0, w0 = 0, h0 = 0;
	Rect2d rect(0, 0, 0, 0);
	for (int i = 0; i<contours_size; i++)
	{
		boundRect[i] = boundingRect((Mat)contours[i]); //查找每个轮廓的外接矩形
		//x0 = boundRect[i].x;  //获得第i个外接矩形的左上角的x坐标
		//y0 = boundRect[i].y; //获得第i个外接矩形的左上角的y坐标
		//w0 = boundRect[i].width; //获得第i个外接矩形的宽度
		//h0 = boundRect[i].height; //获得第i个外接矩形的高度
		//rect = rect | (Rect2d(x0, y0, w0, h0));
		//rectangle(temp_frame, Point(x0, y0), Point(x0 + w0, y0 + h0), Scalar(0, 255, 0), 2, 8); //绘制第i个外接矩形
	}

	return contours.size();
}
/*
输入：形态学处理后的二值图像
输出：1值像素的最大外接矩形
******/
Rect2d printContour(Mat &frame)
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

//初始化时检测目标
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
			

			rect = printContour(result_frame);
			rectangle(temp_frame, rect, Scalar(0, 255, 0), 2, 8); //绘制最大的外接矩形
			//rectangle(result_frame, rect, Scalar(0, 255, 0), 2, 8); //绘制最大的外接矩形
			imshow("Tracking", temp_frame);
			//waitKey(1);
			if(rect.area() >= 50 && rect.area() <= temp_frame.size().area() / 3)
			{
				rect_ = rect;
				break;
			}
			//imshow("result", result_frame);
		}
		//imshow("temp", temp_frame);
		waitKey(1);
	}
	//destroyAllWindows();
	return;
}

//////////////////////////////////
/*
检测运动区域内与原目标最相似的区域中心
输入：原目标，当前帧，运动区域，输出新目标区域
输出：是/否找到目标

计算HS直方图存在问题，可能距离超过1.414
原因：归一化公式错误，已修正
*/
bool getCenter(Mat target, Mat frame, Rect2d rect, Rect2d &newTargetBBox) {

	Point2d wShift = Point2d(target.size().width, 0);
	Point2d hShift = Point2d(0, target.size().height);
	Point2d wShift2 = Point2d(target.size().width / 2, 0);
	Point2d hShift2 = Point2d(0, target.size().height / 2);

	Rect2d tempBox = Rect2d(rect.tl(), target.size());
	Rect2d resuleBox;
	double maxScore = 0;
	double similarThreshold = 0.8;

	//tempBox -= hShift2;
	//tempBox -= wShift2;
	cout << "rect00000: " << rect << endl;
	cout << "tempbox: " << tempBox << endl;
	for (int i = 0; rect.contains(tempBox.tl()); i++)
	{
		for (int j = 0; rect.contains(tempBox.tl()); j++)
		{
			cout << "tempbox: " << tempBox << endl;
			//检查检测框是否不在图片区域内
			if (0 <= tempBox.x  && tempBox.x + tempBox.width <= frame.cols &&
				0 <= tempBox.y  && tempBox.y + tempBox.height <= frame.rows) 
			{
				Mat temp_mat = Mat(frame, tempBox);
				double score = similarScore(target, temp_mat);
				cout << "score: " << score << "\n" << endl;
				if (score > maxScore) {
					maxScore = score;
					resuleBox = Rect2d(tempBox);
				}
			}
			tempBox += hShift;
		}
		tempBox += wShift;
		tempBox.y = rect.y ;//- target.size().height / 2
	}

	if (maxScore > similarThreshold)
	{
		newTargetBBox = Rect2d(resuleBox);
		return true;
	}
	return false;
}
/*
检测运动区域内与原目标最相似的区域中心
输入：原目标，当前帧，运动区域数组(vector)，输出新目标区域
输出：是/否找到目标
*/
bool getCenter1(Mat target, Mat frame, vector<Rect> &boundRect, Rect2d &newTargetBBox) {

	Point2d wShift = Point2d(target.size().width , 0);
	Point2d hShift = Point2d(0, target.size().height);
	Point2d wShift2 = Point2d(target.size().width/2, 0);
	Point2d hShift2 = Point2d(0, target.size().height/2);

	Rect2d resuleBox;
	double maxScore = 0;
	double similarThreshold = 0.75;
	Rect2d tempBox;
	for (int k = 0; k < boundRect.size();++k)
	{
		Rect2d rect = Rect(boundRect[k]);
		if (rect.area() > frame.size().area() / 3)
			continue;

		tempBox = Rect2d(rect.tl(), target.size());
		//tempBox -= hShift2;
		//tempBox -= wShift2;

		//cout << "rect: " << rect << endl;
		for (int i = 0; rect.contains(tempBox.tl()); i++)
		{
			for (int j = 0; rect.contains(tempBox.tl()); j++)
			{
				cout << "tempbox: " << tempBox << endl;
				//检查检测框是否不在图片区域内
				if (0 <= tempBox.x  && tempBox.x + tempBox.width <= frame.cols &&
					0 <= tempBox.y  && tempBox.y + tempBox.height <= frame.rows)
				{
					Mat temp_mat = Mat(frame, tempBox);
					double score = similarScore(target, temp_mat);
					cout << "score: " << score <<"\n"<< endl;
					if (score > maxScore) {
						maxScore = score;
						resuleBox = Rect2d(tempBox);
					}
				}
				tempBox += hShift;
			}
			tempBox += wShift;
			tempBox.y = rect.y;//- target.size().height/2
		}
	}

	if (maxScore > similarThreshold)
	{
		newTargetBBox = Rect2d(resuleBox);
		cout <<"maxScore:"<< maxScore<< endl;
		return true;
	}
	return false;
}
///////////////////////////////////////////////

//跟踪线程程序
void track()
{
	Mat frame;
	Mat  framePre, framePrePre;//frameNow,
	Mat lastTarget,targetBackup;
	//Mat h;
	int dis_threshod = 130;
	int dis_threshod2 = 100;
	int state = 0;
	float fps = 0;
	double timer = 0;
	SYSTEMTIME sys;
	bool ok = false;
	bool flag2 = false;
	string trackerType = "MOSSE";
	Ptr<Tracker> * tracker = new Ptr<Tracker>;
	int detect_fail_time = 0;

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
		framePre = frame.clone();// frameNow.clone();
		//frameNow =

		//处理过程
		//select ROI
		count_frame++;
		if (1 == count_frame)
		{
			// Uncomment the line below to select a different bounding box 
			bbox = selectROI("Tracking", frame, false);

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

			targetBackup = Mat(frame,bbox);
			//imshow("targetBackup",targetBackup);
			(*tracker)->init(frame, bbox);
			
			// Display bounding box. 
			rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
			//putText(frame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
			imshow("Tracking", frame);
		}
		//tracing
		else if(Turning == 0)
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
					circle(frame, center_bbox, 3, Scalar(255, 0, 0), -1, LINE_4, 0);
					putText(frame, "X : " + SSTR(int(center_bbox.x)), Point(300, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					putText(frame, "Y : " + SSTR(int(center_bbox.y)), Point(300, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					/**************
					*      A
					*  5 | 1 | 6
					*  ---------
					*< 3 | 0 | 4 >
					*  ---------
					*  7 | 2 | 8
					*      V
					**************/
					bool s1 = false;
					bool s2 = false;
					bool s3 = false;
					bool s4 = false;
					int temp_s = 0;

					////111111111111111111111111////////////////////
					//if (center_bbox.x - center_0.x > dis_threshod)
					//{
					//	s3 = true;
					//	//sMath;
					//	putText(frame, "<", Point(70, 140), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					//}
					//else if (center_bbox.x - center_0.x < 0- dis_threshod)
					//{
					//	s4 = true;
					//	putText(frame, ">", Point(130, 140), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					//}
					//if (center_bbox.y - center_0.y > dis_threshod)
					//{
					//	s1 = true;
					//	putText(frame, "A", Point(100, 110), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					//}
					//else if (center_bbox.y - center_0.y < 0- dis_threshod)
					//{
					//	s2 = true;
					//	putText(frame, "V", Point(100, 170), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
					//}

					//if (s1&&s3)
					//{
					//	temp_s = 5;
					//}
					//else if (s1&&s4)
					//{
					//	temp_s = 6;
					//}
					//else if (s2&&s3)
					//{
					//	temp_s = 7;
					//}
					//else if (s2&&s4)
					//{
					//	temp_s = 8;
					//}
					//else if (s1)
					//{
					//	temp_s = 1;
					//}
					//else if (s2)
					//{
					//	temp_s = 2;
					//}
					//else if (s3)
					//{
					//	temp_s = 3;
					//}
					//else if (s4)
					//{
					//	temp_s = 4;
					//}

					

					/////22222222222222222222222////////////////
					if (center_bbox.x > frame.cols - dis_threshod2)//turn right
					{
						s3 = true;
					}
					else if (center_bbox.x  < dis_threshod2)//turn left
					{
						s4 = true;
					}
					if (center_bbox.y > frame.rows - dis_threshod2)//turn down
					{
						s1 = true;
					}
					else if (center_bbox.y < dis_threshod2)//turn up
					{
						s2 = true;
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
					if (temp_s != 0) {
						//if (Turning == 0) {
							std::unique_lock<std::mutex> locker1(mu1);//lock
							Situation = temp_s;
							locker1.unlock();//unlock
						//}
							state = 3;
					}
				}
				else
				{
					// Tracking failure detected.
					putText(frame, "1Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
					bboxPre = Rect2d(bbox);
					////检查检测框是否不在图片区域内
					//if (0 <= bboxPre.x  && bboxPre.x + bboxPre.width <= frame.cols &&
					//	0 <= bboxPre.y  && bboxPre.y + bboxPre.height <= frame.rows) {
					//	lastTarget = Mat(framePre, bboxPre);
					//}
					//else {
						lastTarget = Mat(targetBackup);
					/*}*/
					//imshow("last target:",lastTarget);
					cout <<"last target:"<< bboxPre << endl;
					
					detect_fail_time = 0;
					state = 2;

					//std::unique_lock<std::mutex> locker1(mu1);//lock
					//Situation = -2;
					//locker1.unlock();//unlock
				}
				break;

			case 2:
				if (count_frame > 3 && Turning == 0)
				{
//					Mat result_frame = detect3frames(frame, framePre, framePrePre);//frameNow
//					//11111111111111111////////////////////////////
//					//Rect2d rect = printContour(result_frame);
//					//rectangle(frame, rect, Scalar(0, 255, 0), 2, 1);
//					//
//					////检测运动区域大小不合要求
//					//if (!(rect.area() >= 50 && rect.area() <= bboxPre.size().area() * 9 && rect.area() <= frame.size().area() / 3) )
//					//{
//					//	break;
//					//}
//
//					//cout << count_frame << endl;
//					//Point2d center = Point2d((rect.x + rect.width / 2), (rect.y + rect.height / 2));
//					//Size size_bbox = Size(bboxPre.size());
//
//					//Rect2d test_bbox = Rect2d(Point(center.x - 0.5*size_bbox.width, center.y - 0.75*size_bbox.height), size_bbox);
//					////2222222222222222222/////////////////////////////////////
//					//检测运动区域
//					vector<Rect> boundRect;
//					Rect2d test_bbox;
//					printContours(result_frame, boundRect);
//					if (boundRect.empty()) {
//						
//					}
//					//追加1.5倍大小的上个目标位置
//					//Rect2d temp_bbox = Rect2d(bboxPre);
//					//temp_bbox -= Point2d(bboxPre.width / 3, bboxPre.height / 3);
//					//temp_bbox += Size2d(bboxPre.width / 3 * 2, bboxPre.height / 3 * 2);
//					//boundRect.push_back(Rect(temp_bbox));
//					//追加中心区域
//					Rect2d temp_bbox = Rect2d((center_0.x - dis_threshod),
//						(center_0.y - dis_threshod), dis_threshod * 2, dis_threshod * 2);
//					boundRect.push_back(Rect(temp_bbox));
//
//					//运动区域检测目标
//					bool center_exist = getCenter1(lastTarget,frame,boundRect,test_bbox);
//					//存在目标则重新初始化跟踪器
//					if (center_exist) {
//						bbox = Rect2d(test_bbox);
//						
//						delete tracker;
//
//						tracker = new Ptr<Tracker>;
//#if (CV_MINOR_VERSION < 3)
//						{
//							*tracker = Tracker::create(trackerType);
//						}
//#else
//						{
//							*tracker = TrackerMOSSE::create();
//						}
//#endif
//						(*tracker)->init(frame, bbox);
//						rectangle(frame, bbox, Scalar(0, 255, 0), 2, 1);
//						state = 1;
//					}
//					else {
//					/*	detect_fail_time++;
//						if (detect_fail_time > 3)
//							state = 3;*/
//					}
					//1111111111111111111///////////////////////////////
					
					/*bbox = Rect2d(test_bbox);
					state = 3;*/
					//////////////////////////////////////////////
					Rect2d test_bbox;//上一结果检测
					Rect2d temp_bbox = Rect2d(bboxPre);
					temp_bbox -= Point2d(bboxPre.width / 3, bboxPre.height / 3);
					temp_bbox += Size2d(bboxPre.width / 3 * 2, bboxPre.height / 3 * 2);
					bool center_exist = getCenter(lastTarget, frame, temp_bbox, test_bbox);
					if (center_exist) {
						bbox = Rect2d(test_bbox);
												
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
					}
					else
					{
						Mat result_frame = detect3frames(frame, framePre, framePrePre);//frameNow
						Rect2d rect = printContour(result_frame);
						rectangle(frame, rect, Scalar(0, 255, 0), 2, 1);
						////检测运动区域大小不合要求
						//if (!(rect.area() >= 50 && rect.area() <= bboxPre.size().area() * 9 && rect.area() <= frame.size().area() / 3) )
						//{
						//	break;
						//}
						//Point2d center = Point2d((rect.x + rect.width / 2), (rect.y + rect.height / 2));
						//Size size_bbox = Size(bboxPre.size());
						//
						//Rect2d test_bbox = Rect2d(Point(center.x - 0.5*size_bbox.width, center.y - 0.75*size_bbox.height), size_bbox);
						center_exist = getCenter(lastTarget, frame, rect, test_bbox);
						if (center_exist) {
							bbox = Rect2d(test_bbox);

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
						}
					}

				}
			
				rectangle(frame, bboxPre, Scalar(0, 0, 255), 2, 1);
				// Tracking failure detected.
				putText(frame, "2Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);

				break;
			case 3:
				if (count_frame > 3 && Turning == 0)
				{
					Rect2d test_bbox;//中心检测
					Rect2d temp_bbox = Rect2d((center_0.x - dis_threshod),
						(center_0.y - dis_threshod), dis_threshod * 2, dis_threshod * 2);
					bool center_exist = getCenter(targetBackup, frame, temp_bbox, test_bbox);
					if (center_exist) {
						bbox = Rect2d(test_bbox);

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
					}
					else
					{
						Mat result_frame = detect3frames(frame, framePre, framePrePre);//frameNow
						Rect2d rect = printContour(result_frame);
						rectangle(frame, rect, Scalar(0, 255, 0), 2, 1);
						////检测运动区域大小不合要求
						//if (!(rect.area() >= 50 && rect.area() <= bboxPre.size().area() * 9 && rect.area() <= frame.size().area() / 3))
						//{
						//	break;
						//}
						//Point2d center = Point2d((rect.x + rect.width / 2), (rect.y + rect.height / 2));
						//Size size_bbox = Size(bboxPre.size());

						//Rect2d test_bbox = Rect2d(Point(center.x - 0.5*size_bbox.width, center.y - 0.75*size_bbox.height), size_bbox);
						center_exist = getCenter(targetBackup, frame, rect, test_bbox);
						if (center_exist) {
						bbox = Rect2d(test_bbox);

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
						}
					}

				}

				rectangle(frame, bboxPre, Scalar(0, 0, 255), 2, 1);
				// Tracking failure detected.
				putText(frame, "3Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
				
				break;

			default:
				break;
			}

			// Calculate Frames per second (FPS)
			fps = getTickFrequency() / ((double)getTickCount() - timer);

			// Display FPS on frame
			putText(frame, "FPS : " + SSTR(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);


			circle(frame, Point(640, 360), 3, Scalar(255, 255, 0), -1, LINE_4, 0);
			rectangle(frame, Rect2d(100, 100, 1080, 520), Scalar(255, 255, 0), 2, 1);
			rectangle(frame, Rect2d((center_0.x - dis_threshod),(center_0.y- dis_threshod), dis_threshod*2, dis_threshod*2), Scalar(255, 255, 0), 2, 1);
			// Display frame.
			imshow("Tracking", frame);

			// Exit if ESC pressed.
			int k = waitKey(1) & 0xFF;
			if (k == 27)
			{
				break;
			}
		}

		
		GetLocalTime(&sys);
		char timeBuffer[64];
		sprintf_s(timeBuffer, "[%4d/%02d/%02d %02d:%02d:%02d.%03d]", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds);
		putText(frame, "#" + SSTR(int(count_frame))+" "+timeBuffer, Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 255), 2);
		
		string fileName = "";
		fileName += "D:\\images_test\\log_"+SSTR(int(count_frame))+".jpg";
		imwrite(fileName, frame);
		//imshow("temp" , frame);
		//waitKey(1);
	}
	cv::destroyAllWindows();
	delete tracker;
	return;
}


void play()
{
	Mat frame;
	SYSTEMTIME sys;
	int count_frame = 0;
	char timeBuffer[64];
	while (1)
	{
		count_frame++;
		frame = read_Frame();
		//GetLocalTime(&sys);
		//
		//sprintf_s(timeBuffer, "[%4d/%02d/%02d %02d:%02d:%02d.%03d]", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds);
		//putText(frame, "#" + SSTR(int(count_frame)) + " " + timeBuffer, Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 255), 2);
		//circle(frame, Point(640,360), 3, Scalar(255, 255, 0), -1, LINE_4, 0);
		//rectangle(frame, Rect2d(100, 100, 1080, 520), Scalar(255, 255, 0), 2, 1);
		//string fileName = "";
		//fileName += "D:\\images_test\\log_" + SSTR(int(count_frame)) + ".jpg";
		//imwrite(fileName, frame);
		imshow("Tracking", frame);

		// Exit if ESC pressed.
		int k = waitKey(1) & 0xFF;
		if (k == 27)
		{
			break;
		}
	}
	return;
}

int main()
{
	
	//thread reader(write_Frame);
	////Sleep(1);
	//thread tracker(play);

	//thread reader(readFrame);
	////Sleep(1);
	//thread tracker(play);

	//Rect2d rect;
	//thread reader(readFrame);
	////Sleep(1);
	//thread tracker(detect,rect);


	thread reader(readFrame);
	//Sleep(1);
	thread tracker(track);

	reader.join();
	tracker.join();


	system("pause");
	return 0;
}




