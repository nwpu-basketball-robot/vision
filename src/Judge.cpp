#include "Judge.h"

bool protonect_shutdown = false; 

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

bool protonect_paused = false;
libfreenect2::Freenect2Device *devtopause;

void sigusr1_handler(int s)
{
  if (devtopause == 0)
    return;
/// [pause]
  if (protonect_paused)
    devtopause->start();
  else
    devtopause->stop();
  protonect_paused = !protonect_paused;
/// [pause]
}


Judge::Judge(const ros::NodeHandle & n):
    node_(n), initOK(false), isContinue(false), changeModel(false), missCount(0),
    dev(0), pipeline(0), timeLoad(0), model(1)
{
    printf("freenect init begin\n");

    initOK = init();
    if( initOK )
    {  
        //init faster rcnn
        std::string src_path = "/home/huangtairan/catkin_ws/src/vision/data/";
        std::string model_file = src_path + "test/test.pt";
        std::string weights_file = src_path + "test/ZF_faster_rcnn_final.caffemodel";

        int GPUID = 0;
        caffe::Caffe::SetDevice(GPUID);
        caffe::Caffe::set_mode( caffe::Caffe::GPU );
        detector1 = Detector(model_file, weights_file);
        detector2 = Detector(model_file, weights_file);

        //call back
        service_ = node_.advertiseService("visionDate", &Judge::serviceCallBack, this);
        printf("freenect init OK\n");
    }
    else
    {
        printf("freenect init FALSE\n");
    }
}

Judge::~Judge()
{
  dev->stop();
  dev->close();
}

bool Judge::serviceCallBack(basketball_msgs::visionDate::Request  &req,
                            basketball_msgs::visionDate::Response &rep)

{
    this->changeModel   = (this->model != req.model);
    this->model         = req.model; 
    rep.timeLoad        = this->timeLoad;
    rep.balls           = this->balls;
    rep.cylinders       = this->cylinders;

    this->balls.clear();
    this->cylinders.clear();

    if(this->changeModel)
    {
        this->isContinue = false;
        this->missCount  = 0;
    }

    printf("[ serviceCallBack success ]\n");
    return  true;
}

bool Judge::init()
{
    std::string serial = "";

    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));

    MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
    if (filelogger->good())
        libfreenect2::setGlobalLogger(filelogger);
    else
        delete filelogger;
    
    int deviceId = -1;
    if(!pipeline)
    {
 //       pipeline = new libfreenect2::OpenCLPacketPipeline(deviceId);
          pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
    }

    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return false;
    }

    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
    /// [discovery]

    if(pipeline)
    {
    /// [open]
        dev = freenect2.openDevice(serial, pipeline);
    /// [open]
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return false;
    }

    devtopause = dev;

        signal(SIGINT,sigint_handler);
    #ifdef SIGUSR1
        signal(SIGUSR1, sigusr1_handler);
    #endif
    protonect_shutdown = false;

    //阈值初始化
    iLowH    = 60;
    iHighH   = 103;
    iLowS    = 122;
    iHighS   = 239;
    iLowV    = 100;
    iHighV   = 210;
    rectArea = 4000;

    return true;
}

bool Judge::run()
{
    
    if( !initOK )
    {
        printf("can not run, init FALSE\n");
        return false;
    }

    int  types              = 0;
    const bool enable_rgb   = true;
    const bool enable_depth = true;

    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);


    if (enable_rgb && enable_depth)
    {
        if (!dev->start())
        return false;
    }
    else
    {
        if (!dev->startStreams(enable_rgb, enable_depth))
        return false;
    }

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    /// [start]

    /// [registration setup]
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    /// [registration setup]

    size_t framemax = -1;
    size_t framecount = 0;

    cv::Mat colorImage;
    cv::Mat regImage;
    cv::Mat regImageGray;
    cv::Mat regImageTwice;
    /*
    pcl::visualization::CloudViewer viewer_("Point CLoud");
    */
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);

    cv::namedWindow("colorImage",cv::WINDOW_NORMAL);
    cv::namedWindow("regImage",  cv::WINDOW_NORMAL);

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
    while((!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax)) && ros::ok())
    {
        double first_time = cv::getTickCount();

        this->balls.clear();
        this->cylinders.clear();

        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
        {
            std::cout << "timeout!" << std::endl;
            return false;
        }
        
        this->timeLoad = ros::Time::now().toSec();
        
        printf("\n\n%s %lf\n", "timeLoad", this->timeLoad);
        

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        if (enable_rgb && enable_depth)
        {   
            registration->apply(rgb, depth, &undistorted, &registered, true);
        }

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(colorImage);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(regImage);
                
        cv::cvtColor(colorImage, colorImage,    CV_BGRA2BGR);
        cv::cvtColor(regImage,   regImage,      CV_BGRA2BGR);
        cv::cvtColor(regImage,   regImageGray,  CV_BGRA2GRAY);
        cv::resize  (regImage,   regImageTwice, cv::Size(regImage.cols*2, regImage.rows*2) );
        
        if(model == 1)
        {
                               
            if(isContinue && ( missCount > 0 ))
            {
                bool colorDetect = false;
                std::map< int, std::vector< cv::Rect > > results;
                detector1.detect(colorImage, results);

                printf("%s\n", "Ball Color Image Find Result");
                for(int i = 1; i<=5; i++)
                {
                    std::vector< cv::Rect > &result = results.at(i);
                    for(int j = 0; j < result.size(); j++)
                    {
                      if(i == 1)
                            cv::rectangle(colorImage, result[j], cv::Scalar(0, 0, 255), 4);
                      else if(i == 2)
                            cv::rectangle(colorImage, result[j], cv::Scalar(0, 255, 0), 4);
                      else if(i == 3)
                            cv::rectangle(colorImage, result[j], cv::Scalar(0, 255, 0), 4);
                      else if(i==4)
                            cv::rectangle(colorImage, result[j], cv::Scalar(0, 255, 0), 4);
                      else 
                            cv::rectangle(colorImage, result[j], cv::Scalar(0, 0, 255), 4);
                        colorDetect = true;

                        basketball_msgs::ballElem    ball;
                        if(i==1||i==5){
                            ball.type      = 1;
                        }
                        else{
                            ball.type      = 2;
                        }
                        ball.distance      = -1;
                        ball.theta         =  computeTheta( cv::Point(result[j].x + result[j].width/2, result[j].y + result[j].height/2), 
                                                            dev->getColorCameraParams().fx,
                                                            dev->getColorCameraParams().cx);
                        this->balls.push_back(ball);

                        printf("%s %d\n",  "[Color] type     value: ", ball.type);
                        printf("%s %lf\n", "[Color] theta    value: ", ball.theta);
                        printf("%s %lf\n", "[Color] distance value: ", ball.distance);
                    }
                }

                if( colorDetect )
                {
                    missCount--;
                    isContinue = (missCount == 0) ? (false) : (true);
                }
                else
                {
                    missCount++;
                    missCount =  (missCount > 3) ? (3) : (missCount);
                }
            }
            else
            {
                bool regDetect = false;
                std::map<int, std::vector< cv::Rect > > results;
                detector2.detect(regImageTwice, results);

                printf("%s\n", "Ball Reg Image Find Result");
                for( int i = 1; i <= 5; i++)
                {
                    std::vector< cv::Rect >  &result = results.at(i);

                    for(int j = 0; j < result.size(); j++)
                    {
                        cv::Rect rect;
                        rect.x      = result[j].x / 2;
                        rect.y      = result[j].y / 2;
                        rect.width  = result[j].width / 2;
                        rect.height = result[j].height / 2;

                         if(i == 1)
                            cv::rectangle(regImage, rect, cv::Scalar(0, 0, 255), 4);
                        else if(i == 2)
                            cv::rectangle(regImage, rect, cv::Scalar(0, 255, 0), 4);
                            else if(i == 3)
                            cv::rectangle(regImage, rect, cv::Scalar(0, 255, 0), 4);
                            else if(i==4)
                            cv::rectangle(regImage, rect, cv::Scalar(0, 255, 0), 4);
                            else 
                            cv::rectangle(regImage, rect, cv::Scalar(0, 0, 255), 4);
                    
                        regDetect = true;

                        basketball_msgs::ballElem   ball;
                        if(i==1||i==5){
                            ball.type      = 1;
                        }
                        else{
                            ball.type      = 2;
                        }
                        ball.distance      = -1;
                        ball.theta         = computeTheta( cv::Point(rect.x + rect.width/2, rect.y + rect.height/2), 
                                                           dev->getIrCameraParams().fx,
                                                           dev->getIrCameraParams().cx);
                        
                        getPointCloud(undistorted, registration, rect, regImageGray, cloud_ptr);

                        PointT                       center;
                        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
                        bool judgePCL = pointCloudJudge.judgeCenter(cloud_ptr, cloud_filtered, center);
                       /*
                        viewer_.showCloud(cloud_filtered);
                        */

                        if( judgePCL )
                        {
                            ball.distance = center.z;
                            ball.theta    = atan2(center.x, center.z);
                        }
                        this->balls.push_back( ball );
                    
                        printf("%s %d\n",  "[Reg] type     value: ", ball.type);
                        printf("%s %lf\n", "[Reg] theta    value: ", ball.theta);
                        printf("%s %lf\n", "[Reg] distance value: ", ball.distance);
                    }
                }

                if( !regDetect )
                    missCount++;

                isContinue = (missCount >= 3) ? (true) : (false);

            }
        }
        else if(model == 2)
        {
            findCylinder.setParameters(iLowH, iHighH, 
                                       iLowS, iHighS, 
                                       iLowV, iHighV,
                                       rectArea);
            
            cv::Rect  regRect;
            cv::Point regCenter;

            bool judgeCylinder = findCylinder.run(regImageTwice, regCenter, regRect);

            printf("%s\n", "Cylinder Find Result");
            if( judgeCylinder )
            {
                regCenter.x    = regCenter.x / 2;
                regCenter.y    = regCenter.y / 2;
        
                regRect.x      = regRect.x / 2;
                regRect.y      = regRect.y / 2;
                regRect.height = regRect.height / 2;
                regRect.width  = regRect.width  / 2;
                
                basketball_msgs::cylinderElem cylinder;

                cylinder.distance   = -1;
                cylinder.theta      = computeTheta(regCenter, dev->getIrCameraParams().fx,
                                                   dev->getIrCameraParams().cx);
                
                cv::rectangle(regImage, regRect, cv::Scalar(0,255,255), 2);

                getPointCloud(undistorted, registration, regRect, regImageGray, cloud_ptr);
                
                PointT                       center;
                pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
                bool judgePCL = pointCloudJudge.judgeCenter(cloud_ptr, cloud_filtered, center);
                
                /*
                viewer_.showCloud(cloud_filtered);
                */

                if( judgePCL )
                {
                    cylinder.distance = center.z;
                    cylinder.theta    = atan2(center.x, center.z);
                }

                this->cylinders.push_back(cylinder);

                printf("%s %lf\n", "[Cylinder] theta    value: ",  cylinder.theta);
                printf("%s %lf\n", "[Cylinder] distance value: ",  cylinder.distance);
            }
        }

        double second_time = cv::getTickCount();
        printf("time per loop: %lf\n\n", (second_time - first_time)/cv::getTickFrequency());

        cv::imshow("colorImage", colorImage);
        cv::imshow("regImage", regImage);
        cv::waitKey(1);

        rate.sleep();
        ros::spinOnce();

        listener.release(frames);
  }

  delete registration;
  return true;
}

double Judge::computeTheta(const cv::Point & center, 
                           const double    & fx,
                           const double    & cx)
{
    if(fx < 0 || cx < 0)
    {
        return 0;
    }
    else{
        return atan2(center.x - cx, fx);
    }
}

void Judge::getPointCloud(const libfreenect2::Frame       & undistorted,
                          const libfreenect2::Registration* registration,
                          const cv::Rect                  & rect,
                          const cv::Mat                   & regImageGray,
                          pcl::PointCloud<PointT>::Ptr    & cloud_ptr)
{
    const int min_r = rect.y;
    const int min_c = rect.x;
    const int max_r = rect.y + rect.height;
    const int max_c = rect.x + rect.width;

    cloud_ptr->clear();
    for( int r = min_r; r<max_r; r++)
    {
        const uchar* regPtr = regImageGray.ptr<uchar>(r);
        for(int c = min_c; c<max_c; c++)
        {
            if(regPtr[c] == 0)
            {
                continue;
            }
            else
            {
                PointT p;
                registration -> getPointXYZ ( &undistorted, r, c, p.x, p.y, p.z);
                cloud_ptr->points.push_back(p);
            }
        }
    }
    
    cloud_ptr->width = cloud_ptr->points.size ();
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = true;
}