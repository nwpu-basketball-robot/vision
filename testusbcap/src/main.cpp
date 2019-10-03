#include "usb_capture_with_thread.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "linux/videodev2.h"

#include<opencv2/opencv.hpp>

using namespace cv;
int main()
{
	wmj::UsbCaptureWithThread vdo("/dev/video0",1280,720);
	//vdo.open(1);
	Mat img;
	//resize(img, img, Size(1280,720), 0, 0, INTER_LINEAR);
	while(1)
	{
		vdo>>img;
		imshow("img",img);
		waitKey(30);
		if(waitKey(3)==27)break;
	}
}