#include "stdafx.h"

#include "matList.h"



using namespace std;
using namespace cv;

void matList::pushNewFrame(Mat newFrame)
{
	std::unique_lock<std::mutex> locker(mu);//lock
	if (MAX_SIZE == g_frameList.size())
		g_frameList.pop_front();
	g_frameList.push_back(newFrame);

	if (MIN_BUFFUR_SIZE < g_frameList.size())
		cond.notify_one();//unlock
	else
		locker.unlock();//unlock
}
	
Mat matList::getNewFrame()
{
	Mat temp_frame;

	std::unique_lock<std::mutex> locker(mu);//lock
	while (g_frameList.empty())
	{
		cond.wait(locker);
	}

	list<Mat>::iterator it;
	it = g_frameList.end();
	it--;
	temp_frame = *it;
		
	locker.unlock();//unlock

	return temp_frame;

}

matList mat_list;