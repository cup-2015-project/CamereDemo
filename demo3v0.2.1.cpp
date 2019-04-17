// demo3.cpp : �������̨Ӧ�ó������ڵ㡣
//˫�̲߳��ż������ͷ���߳�һ��������ͷ��ȡ֡��Mat���У��̶߳���Mat���ж�ȡ֡��ʾ����
//����ʵʱ���ŵİ汾������ҳ��ؽ���ʱ������첻��

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
////////ϵ�лص�����

//--------------------------------------------
int iPicNum = 0;//Set channel NO.
LONG nPort = -1;
HWND hWnd = NULL;



// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


//����ص� ��ƵΪYUV����(YV12)����ƵΪPCM����
void CALLBACK DecCBFun(long nPort, char * pBuf, long nSize, FRAME_INFO * pFrameInfo, long nReserved1, long nReserved2)
{
	long lFrameType = pFrameInfo->nType;

	FrameHeight = pFrameInfo->nHeight;//ȫ�ֱ�����¼֡����
	FrameWidth = pFrameInfo->nWidth;

	if (lFrameType == T_YV12)
	{
		Mat pImg_YUV(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, pBuf);
		//ֱ�ӽ�YV12��ʽ��bufferת��Mat��д��ȫ��������
		std::unique_lock<std::mutex> locker(mu);//lock

		if (MAX_SIZE == g_frameList.size())
			g_frameList.pop_front();
		//���洦��
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


///ʵʱ���ص�
void CALLBACK fRealDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *pUser)
{
	DWORD dRet;
	switch (dwDataType)
	{
	case NET_DVR_SYSHEAD:    //ϵͳͷ
		if (!PlayM4_GetPort(&nPort)) //��ȡ���ſ�δʹ�õ�ͨ����
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
			//���ý���ص����� ֻ���벻��ʾ
			if (!PlayM4_SetDecCallBack(nPort, DecCBFun))
			{
				dRet = PlayM4_GetLastError(nPort);
				break;
			}

			//���ý���ص����� ��������ʾ
			//if (!PlayM4_SetDecCallBackEx(nPort,DecCBFun,NULL,NULL))
			//{
			//	dRet=PlayM4_GetLastError(nPort);
			//	break;
			//}

			//����Ƶ����
			if (!PlayM4_Play(nPort, hWnd))
			{
				dRet = PlayM4_GetLastError(nPort);
				break;
			}

			//����Ƶ����, ��Ҫ�����Ǹ�����
			//if (!PlayM4_PlaySound(nPort))
			//{
			//	dRet = PlayM4_GetLastError(nPort);
			//	break;
			//}
		}
		break;

	case NET_DVR_STREAMDATA:   //��������
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
	case EXCEPTION_RECONNECT:    //Ԥ��ʱ����
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
	// ��ʼ��
	NET_DVR_Init();
	//��������ʱ��������ʱ��
	NET_DVR_SetConnectTime(2000, 1);
	NET_DVR_SetReconnect(10000, true);

	//---------------------------------------
	// ��ȡ����̨���ھ��
	//HMODULE hKernel32 = GetModuleHandle((LPCWSTR)"kernel32");
	//GetConsoleWindow = (PROCGETCONSOLEWINDOW)GetProcAddress(hKernel32,"GetConsoleWindow");

	//---------------------------------------
	// ע���豸
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
	//�����쳣��Ϣ�ص�����
	NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);


	//cvNamedWindow("IPCamera");
	//---------------------------------------
	//����Ԥ�������ûص������� 
	NET_DVR_CLIENTINFO ClientInfo;
	ClientInfo.lChannel = 1;        //Channel number �豸ͨ����
	ClientInfo.hPlayWnd = NULL;     //����Ϊ�գ��豸SDK������ֻȡ��
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
	//�ر�Ԥ��
	if (!NET_DVR_StopRealPlay(lRealPlayHandle))
	{
		printf("NET_DVR_StopRealPlay error! Error number: %d\n", NET_DVR_GetLastError());
		return;
	}

	//ע���û�
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



	while (1)
	{
		//ȡ��֡
		std::unique_lock<std::mutex> locker(mu);//lock
		while (g_frameList.empty() )
		{
			cond.wait(locker);
		}
		if (i > TOTAL - 1)//ȷ����������������ֹ֡ͣ�����ռ�֡�������޷����Ѵ������
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


		//ȡ��YV12��ʽ��Mat����ת������ʾ
		Mat pImg(FrameHeight, FrameWidth, CV_8UC3);
		//Mat pImg_YUV(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, pBuf);
		Mat pImg_YCrCb(FrameHeight, FrameWidth, CV_8UC3);
		cvtColor(temp_frame, pImg, CV_YUV2BGR_YV12);
		cvtColor(pImg, pImg_YCrCb, CV_BGR2YCrCb);

		//��ʼ����
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

			if (ok)
			{
				// Tracking success : Draw the tracked object
				rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
			}
			else
			{
				// Tracking failure detected.
				putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
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

