// demo3.cpp : 定义控制台应用程序的入口点。
//双线程播放监控摄像头，线程一调用摄像头读取帧到Mat队列，线程二从Mat队列读取帧显示播放
//可以实时播放的版本，与网页监控界面时间戳差异不大

#include "stdafx.h"
#include "Windows.h"
#include "stdio.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include "Windows.h"
#include "HCNetSDK.h"
#include "plaympeg4.h"
using namespace std;
using namespace cv;


#define MAX_SIZE 50
//#define TOTAL 1000
#define DROP 5

#define USECOLOR 1

//CRITICAL_SECTION g_cs_frameList;
std::list<Mat> g_frameList;

int i = -1;
int FrameWidth;
int FrameHeight;
mutex mu;
condition_variable cond;

int Situation = 0;
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
		std::unique_lock<std::mutex> locker(mu);//lock

		if (MAX_SIZE == g_frameList.size())
			g_frameList.pop_front();
		//缓存处理
		g_frameList.push_back(pImg_YUV);
		++i;
		//cout << "read:" << i << endl;
		//cout << "size:" << g_frameList.size() << endl;

		if (DROP < g_frameList.size())
			cond.notify_one();//unlock
		else
			locker.unlock();//unlock
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
	int speed =2;
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
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_DOWN, 0, speed);
			cout << "start A"<< endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_DOWN, 1, speed);
			cout << "stop A" << endl;
			//stop
			break;
		case 2:
			//move
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 0, speed);
			cout << "start V" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, TILT_UP, 1, speed);
			cout << "stop V" << endl;
			//stop
			break;
		case 3:
			//move  PAN_RIGHT
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 0, speed);
			cout << "start <" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_RIGHT, 1, speed);
			cout << "stop <" << endl;
			//stop
			break;
		case 4:
			//move  PAN_LEFT
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_LEFT, 0, speed);
			cout << "start >" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, PAN_LEFT, 1, speed);
			cout << "stop >" << endl;
			//stop
			break;
		case 5:
			//move  DOWN_RIGHT
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_RIGHT, 0, speed);
			cout << "start 5" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_RIGHT, 1, speed);
			cout << "stop 5" << endl;
			//stop
			break;
		case 6:
			//move  DOWN_LEFT
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_LEFT, 0, speed);
			cout << "start 6" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, DOWN_LEFT, 1, speed);
			cout << "stop 6" << endl;
			//stop
			break;
		case 7:
			//move  UP_RIGHT
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 0, speed);
			cout << "start 7" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_RIGHT, 1, speed);
			cout << "stop 7" << endl;
			//stop
			break;
		case 8:  
			//move  UP_LEFT
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_LEFT, 0, speed);
			cout << "start 8" << endl;
			Sleep(100);
			NET_DVR_PTZControlWithSpeed(lRealPlayHandle, UP_LEFT, 1, speed);
			cout << "stop 8" << endl;
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
	cout <<"turn end" << endl;
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

void track()
{
	Mat temp_frame;
	Mat frame;

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
	Rect2d bbox(287, 23, 86, 320);
	Rect2d image0;


	while (1)
	{
		//取出帧
		std::unique_lock<std::mutex> locker(mu);//lock
		while (g_frameList.empty() )
		{
			cond.wait(locker);
		}
		//if (i > TOTAL - 1)//确定结束条件，处理帧停止后还需收集帧，否则无法唤醒处理程序
		//{
		//	locker.unlock();//unlock
		//	destroyAllWindows();
		//	return;
		//}

		//cout << "write:" << i << endl;
		list<Mat>::iterator it;
		it = g_frameList.end();
		it--;
		temp_frame = *it;

		locker.unlock();//unlock


		//取出YV12格式的Mat进行转换、显示
		Mat pImg(FrameHeight, FrameWidth, CV_8UC3);
		//Mat pImg_YUV(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, pBuf);
		Mat pImg_YCrCb(FrameHeight, FrameWidth, CV_8UC3);
		cvtColor(temp_frame, pImg, CV_YUV2BGR_YV12);
		cvtColor(pImg, pImg_YCrCb, CV_BGR2YCrCb);

		//开始处理
		frame = pImg;
		count_frame++;
		if (1 == count_frame)
		{
			// Uncomment the line below to select a different bounding box 
			bbox = selectROI(frame, false);

			// Display bounding box. 
			rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
			imshow("Tracking", frame);
			tracker->init(frame, bbox);
		}
		else
		{
			// Start timer
			double timer = (double)getTickCount();

			// Update the tracking result
			bool ok = tracker->update(frame, bbox);

			// Calculate Frames per second (FPS)
			float fps = getTickFrequency() / ((double)getTickCount() - timer);

			Point tl = bbox.tl();
			Point br = bbox.br();
			Point center_bbox = (br - tl) / 2 + tl;

			image0 = getWindowImageRect("Tracking");
			Point tl0 = image0.tl();
			Point br0 = image0.br();
			Point center_0 = (br0 - tl0) / 2;

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
				////////////
			}
			else
			{
				// Tracking failure detected.
				putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);

				std::unique_lock<std::mutex> locker1(mu1);//lock
				Situation = -2;
				locker1.unlock();//unlock
			}

			// Display tracker type on frame
			putText(frame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

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

		//imshow("temp", pImg);
		//waitKey(1);
	}
	//destroyAllWindows();
	count_frame = 0;
	std::unique_lock<std::mutex> locker1(mu1);//lock
	Situation = -1;
	locker1.unlock();//unlock
	return;
}

int main()
{
	thread reader(readFrame);
	//Sleep(10);
	thread tracker(track);
	tracker.join();
	//cout << "tracker finished" << endl;
	//thread tracker1(track);
	//tracker1.join();
	
	reader.join();
	
	system("pause");
	return 0;
}

