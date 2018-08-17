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

Judge::Judge(const ros::NodeHandle &n) : node_(n), initOK(false), isContinue(false), changeModel(false), missCount(0),
                                         dev(0), pipeline(0), timeLoad(0), model(1)
{
    ROS_INFO("freenect init begin\n");
    initOK = init();
    if (initOK)
    {
        //init ssd model
        std::string src_path = "/home/shirlim/catkin_shirlim/src/bvision/data/";
        //载入模型权重
        std::string model_file = src_path + "deploy.prototxt";
        std::string weights_file = src_path + "VGG_VOC0712_SSD_480x270_iter_32000.caffemodel";

        detector = new Detector(model_file, weights_file, "", "104,117,123");

        //call back
        service_ = node_.advertiseService("visionDate", &Judge::serviceCallBack, this);
        ROS_INFO("freenect init OK\n");
    }
    else
    {
        ROS_INFO("freenect init FALSE\n");
    }
}

Judge::~Judge()
{
    dev->stop();
    dev->close();
}

bool Judge::serviceCallBack(basketball_msgs::visionDate::Request &req,
                            basketball_msgs::visionDate::Response &rep)
{

    std::cout << "modle=" << req.model << std::endl;

    this->changeModel = (this->model != req.model);
    this->model = req.model;
    //return the prediction
    rep.timeLoad = this->timeLoad;
    rep.balls = this->balls;
    rep.cylinders = this->cylinders;

    this->balls.clear();
    this->cylinders.clear();

    if (this->changeModel)
    {
        this->isContinue = false;
        this->missCount = 0;
    }

    std::cout << "[ serviceCallBack success ]" << std::endl;
    return true;
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
    if (!pipeline)
    {
        // pipeline = new libfreenect2::OpenCLPacketPipeline(deviceId);
        pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
    }

    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return false;
    }

    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
    /// [discovery]

    if (pipeline)
    {
        /// [open]
        dev = freenect2.openDevice(serial, pipeline);
        /// [open]
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if (dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return false;
    }

    devtopause = dev;

    signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
    signal(SIGUSR1, sigusr1_handler);
#endif
    protonect_shutdown = false;

    // init basketball column HSV value

    // // init yellow column HSV value
    // iLowH_Yellow_Column = 0;
    // iHighH_Yellow_Column = 179;
    // iLowS_Yellow_Column = 0;
    // iHighS_Yellow_Column = 255;
    // iLowV_Yellow_Column = 0;
    // iHighV_Yellow_Column = 255;
    // rectArea_Yellow_Column = 4000;

    return true;
}

bool Judge::run()
{

    if (!initOK)
    {
        ROS_INFO("can not run, init FALSE\n");
        return false;
    }

    int types = 0;
    const bool enable_rgb = true;
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
    libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    /// [registration setup]

    size_t framemax = -1;
    size_t framecount = 0;

    cv::Mat colorImage;
    cv::Mat regImage;
    cv::Mat regImageGray;

    // pcl::visualization::CloudViewer viewer_("Point CLoud");

    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);

    cv::namedWindow("regImage", cv::WINDOW_NORMAL);


    iLowH_Basketball_Column = 36;
    iHighH_Basketball_Column = 144;
    iLowS_Basketball_Column = 114;
    iHighS_Basketball_Column = 238;
    iLowV_Basketball_Column = 70;
    iHighV_Basketball_Column = 231;
    rectArea_Basketball_Column = 4000;


    //for Debug
    //定义了一个窗口，窗口中有6个滑动条，通过滑动滑动条，可以改变阈值的大小，如果要分割其他颜色，就可以使用滑动条
    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH_Basketball_Column, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH_Basketball_Column, 179);
    cvCreateTrackbar("LowS", "Control", &iLowS_Basketball_Column, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS_Basketball_Column, 255);
    cvCreateTrackbar("LowV", "Control", &iLowV_Basketball_Column, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV_Basketball_Column, 255);
    //for Debug

    ros::Rate rate(30);

    while ((!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax)) && ros::ok())
    {
        this->balls.clear();
        this->cylinders.clear();

        //获取图像超时
        if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
        {
            std::cout << "timeout!" << std::endl;
            return false;
        }

        this->timeLoad = ros::Time::now().toSec();

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        if (enable_rgb && enable_depth)
        {
            registration->apply(rgb, depth, &undistorted, &registered, true);
        }

        //rgb image
        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(colorImage);
        //reg image
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(regImage);
        //regimage 512*424
        cv::cvtColor(colorImage, colorImage, CV_BGRA2BGR);
        cv::cvtColor(regImage, regImage, CV_BGRA2BGR);
        cv::cvtColor(regImage, regImageGray, CV_BGRA2GRAY);

        //对于球体识别
        if (model == 1)
        {
            //之所以要用regImage，是因为从原图像变换到深度图之后经过解算，该图像并不是成一定比例缩放到深度图，所以这里直接使用了深度图进行计算。
            bool regDetect = false;
            std::vector<std::vector<float>> results;
            std::vector<cv::Rect> target_rects;
            results = detector->Detect(regImage(cv::Rect(0, 185, 512, 239)));
            ROS_INFO("%s\n", "Ball Reg Image Find Result");
            for (int i = 0; i < results.size(); i++)
            {
                std::vector<float> rect = results[i];
                if (rect[2] < 0.8)
                    continue;
                cv::Rect cv_rect(cv::Point(rect[3] * regImage.cols, rect[4] * 239),
                                 cv::Point(rect[5] * regImage.cols, rect[6] * 239));
                cv_rect.y += 185;
                target_rects.push_back(cv_rect);
                cv::rectangle(regImage, cv_rect, cv::Scalar(255, 0, 0), 2);
                basketball_msgs::ballElem ball;
                switch (int(rect[1]))
                {
                //basketball
                case 1:
                case 2:
                {
                    ball.type = 1;
                    cv::putText(regImage, "basketball", cv_rect.tl(), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 255), 2);
                }
                break;
                //valleyball
                case 3:
                case 4:
                {
                    ball.type = 2;
                    cv::putText(regImage, "valleyball", cv_rect.tl(), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 255), 2);
                }
                break;
                }
                ball.distance = -1;
                ball.theta = computeTheta(cv::Point(cv_rect.x + cv_rect.width / 2, cv_rect.y + cv_rect.height / 2),
                                          dev->getIrCameraParams().fx,
                                          dev->getIrCameraParams().cx);

                getPointCloud(undistorted, registration, cv_rect, regImageGray, cloud_ptr);
                PointT center;
                pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
                bool judgePCL = pointCloudJudge.judgeCenter(cloud_ptr, cloud_filtered, center);
                // viewer_.showCloud(cloud_filtered);
                ball.distance = center.z;

                balls.push_back(ball);
                // ROS_INFO("%s %d\n", "[IR] type      value: ", ball.type);
                // ROS_INFO("%s %lf\n", "[IR] theta    value: ", ball.theta);
                // ROS_INFO("%s %lf\n", "[IR] distance value: ", ball.distance);
            }

            if (!regDetect)
                missCount++;
            isContinue = (missCount >= 3) ? (true) : (false);
            // }
        }
        //对于定位柱识别
        else if (model == 2)
        {
            cv::Mat regImageTwice;
            cv::resize(regImage, regImageTwice, cv::Size(regImage.cols * 2, regImage.rows * 2));

            findCylinder.setParameters(iLowH_Basketball_Column, iHighH_Basketball_Column,
                                       iLowS_Basketball_Column, iHighS_Basketball_Column,
                                       iLowV_Basketball_Column, iHighV_Basketball_Column,
                                       rectArea_Basketball_Column);
            cv::Rect regRect;
            cv::Point regCenter;

            bool judgeCylinder = findCylinder.run(regImageTwice, regCenter, regRect);

            ROS_INFO("%s\n", "Column Find Result");
            if (judgeCylinder)
            {
                regCenter.x = regCenter.x / 2;
                regCenter.y = regCenter.y / 2;

                regRect.x = regRect.x / 2;
                regRect.y = regRect.y / 2;
                regRect.height = regRect.height / 2;
                regRect.width = regRect.width / 2;

                basketball_msgs::cylinderElem cylinder;

                cylinder.distance = -1;
                cylinder.theta = computeTheta(regCenter, dev->getIrCameraParams().fx,
                                              dev->getIrCameraParams().cx);

                cv::rectangle(regImage, regRect, cv::Scalar(0, 255, 255), 2);

                getPointCloud(undistorted, registration, regRect, regImageGray, cloud_ptr);

                PointT center;
                pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
                bool judgePCL = pointCloudJudge.judgeCenter(cloud_ptr, cloud_filtered, center);

                if (judgePCL)
                {
                    cylinder.distance = center.z;
                    // cylinder.theta = atan2(center.x, center.z);
                }

                this->cylinders.push_back(cylinder);

                ROS_INFO("%s %lf\n", "[Cylinder] theta    value: ", cylinder.theta);
                ROS_INFO("%s %lf\n", "[Cylinder] distance value: ", cylinder.distance);
            }
        }
        // cv::imshow("colorImage", colorImage);
        cv::imshow("regImage", regImage);
        cv::waitKey(1);

        rate.sleep();
        ros::spinOnce();
        listener.release(frames);
    }

    delete registration;
    return true;
}

double Judge::computeTheta(const cv::Point &center,
                           const double &fx,
                           const double &cx)
{
    if (fx < 0 || cx < 0)
    {
        return 0;
    }
    else
    {
        return atan2(center.x - cx, fx);
    }
}

void Judge::getPointCloud(const libfreenect2::Frame &undistorted,
                          const libfreenect2::Registration *registration,
                          const cv::Rect &rect,
                          const cv::Mat &regImageGray,
                          pcl::PointCloud<PointT>::Ptr &cloud_ptr)
{
    const int min_r = rect.y;
    const int min_c = rect.x;
    const int max_r = rect.y + rect.height;
    const int max_c = rect.x + rect.width;

    cloud_ptr->clear();
    for (int r = min_r; r < max_r; r++)
    {
        const uchar *regPtr = regImageGray.ptr<uchar>(r);
        for (int c = min_c; c < max_c; c++)
        {
            if (regPtr[c] == 0)
            {
                continue;
            }
            else
            {
                PointT p;
                registration->getPointXYZ(&undistorted, r, c, p.x, p.y, p.z);
                cloud_ptr->points.push_back(p);
            }
        }
    }

    cloud_ptr->width = cloud_ptr->points.size();
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle node;

    Judge judge(node);
    judge.run();

    return 0;
}
