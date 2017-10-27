
#include "camVision.h"

CamVision::CamVision(const ros::NodeHandle & n):
    node_(n), timeLoad(0)
{
    init();
    service_ = node_.advertiseService("camVisionDate", &CamVision::serviceCallBack, this);
}

CamVision::~CamVision()
{

}

bool CamVision::serviceCallBack(basketball_msgs::camVisionDate::Request  &req,
                                basketball_msgs::camVisionDate::Response &rep)

{
    rep.timeLoad        = this->timeLoad;
    rep.cylinders       = this->cylinders;

    this->cylinders.clear(); 

    printf("[ serviceCallBack success ]\n");
    return  true;
}

void CamVision::init()
{
    iLowH    = 60;
    iHighH   = 103;
    iLowS    = 122;
    iHighS   = 239;
    iLowV    = 100;
    iHighV   = 210;
    rectArea = 3500;

    fx       = 795.404;
    cx       = 300;
}

double CamVision::computeTheta(const cv::Point & center)
{
    if(fx < 0 || cx < 0)
    {
        return 0;
    }
    else{
        return atan2(center.x - cx, fx);
    }
}

bool CamVision::run()
{
    cv::Mat          img;
    cv::VideoCapture cap(1);

    /*
    //for Debug
    //定义了一个窗口，窗口中有6个滑动条，通过滑动滑动条，可以改变阈值的大小，如果要分割其他颜色，就可以使用滑动条
    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);
    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);
    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
    //for Debug
    */
    

    ros::Rate rate(30);
    while( cap.isOpened() && ros::ok())
    {
        double first_time = cv::getTickCount();

        this->cylinders.clear();
        this->timeLoad = ros::Time::now().toSec();
        
        cap >> img;

        findCylinder.setParameters(iLowH, iHighH, 
                                   iLowS, iHighS, 
                                   iLowV, iHighV,
                                   rectArea);
        
        cv::Rect     rect;
        cv::Point    center;
        bool judge = findCylinder.run(img, center, rect);

        if( judge )
        {
            basketball_msgs::cylinderElem cylinder;
            
            cylinder.distance   = -1;
            cylinder.theta      = -computeTheta(center);

            cv::rectangle(img, rect, cv::Scalar(0,0,255), 2);

            this->cylinders.push_back(cylinder);

            printf("%s %lf\n", "[Cylinder] theta    value: ",  cylinder.theta);
            printf("%s %lf\n", "[Cylinder] distance value: ",  cylinder.distance);
        }

        double second_time = cv::getTickCount();
        printf("time per loop: %lf\n\n", (second_time - first_time)/cv::getTickFrequency());

        cv::imshow("CamVision", img);
        cv::waitKey(1);

        rate.sleep();
        ros::spinOnce();
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_vision");
    ros::NodeHandle node;

    CamVision judge(node);
    judge.run();

    return 0;
}