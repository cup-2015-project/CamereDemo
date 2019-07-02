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


#define MAX_SIZE 100
#define TOTAL 500
#define DROP 5

#define USECOLOR 1

//CRITICAL_SECTION g_cs_frameList;
std::list<Mat> g_frameList;
list<int> testList;
int i = -1;
int FrameWidth;
int FrameHeight;
mutex mu;
condition_variable cond;


///////////////////////////////////
////////系列回调函数

//--------------------------------------------
int iPicNum = 0;//Set channel NO.
LONG nPort = -1;
HWND hWnd = NULL;






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
		cout << "read:" << i << endl;
		cout << "size:" << g_frameList.size() << endl;

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
	while (1)
	{
		std::unique_lock<std::mutex> locker(mu);//lock
		while (g_frameList.empty() )
		{
			cond.wait(locker);
		}
		if (i > TOTAL - 1)//确定结束条件，处理帧停止后还需收集帧，否则无法唤醒处理程序
		{
			locker.unlock();//unlock
			destroyAllWindows();
			return;
		}

		cout << "write:" << i << endl;
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

		imshow("temp", pImg);
		waitKey(1);
	}
	//destroyAllWindows();
	return;
}

int main()
{
	thread reader(readFrame);
	//Sleep(10);
	thread tracker(track);


	reader.join();
	tracker.join();

	system("pause");
	return 0;
}

