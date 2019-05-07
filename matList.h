#pragma once

#include "stdafx.h"

#include <list>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>



using namespace std;
using namespace cv;

#define MAX_SIZE 70
#define MIN_BUFFUR_SIZE 5
#define TOTAL 500

class matList
{
public:
	std::list<Mat> g_frameList;
	mutex mu;
	condition_variable cond;


	void pushNewFrame(Mat newFrame);
	Mat getNewFrame();
	
};