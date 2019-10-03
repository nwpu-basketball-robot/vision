////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      UsbCapture Code for robot
///ALL RIGHTS RESERVED
///@file:usb_capture_with_opencv.h
///@brief: 由于采集图像，摄像头传输数据需要一定时间，单独开启线程采集图像，采用生产者模式，
/// 进行图像采集。
///@vesion 1.0
///@author: gzr,gezp
///@email: 1350824033@qq.com
///@date: 18-7-25
///修订历史：
///采用opencv接口进行摄像头图像采集，可以达到较高的帧率。但是没有调整参数的接口
////////////////////////////////////////////////////////////////////////////////

#ifndef LIB_USB_CAPTURE_WITH_OPENCV_H
#define LIB_USB_CAPTURE_WITH_OPENCV_H

#include <opencv2/opencv.hpp>
#include "base_thread.h"


class UsbCaptureWithOpenCV:public BaseThread{
public:
    UsbCaptureWithOpenCV();
    ~UsbCaptureWithOpenCV();
private:
    std::string mVideoPath;
    cv::VideoCapture mCap;
    bool mIsInit;
    bool mIsOpen;
    cv::Mat mImgTmp,mImg;//缓存的图像

    //线程标志，消费者和生产者模型
    bool mIsUpdate;
    pthread_mutex_t imgMutex = PTHREAD_MUTEX_INITIALIZER; //互斥变量锁
    //相机图像分辨率
    int mCaptureWidth;
    int mCaptureHeight;
    //unsigned int mImgFormat;//获取图像格式，和摄像头相关
    bool mIsChangeFormat;
    bool mIsCapture;

private:
    /** 相机初始化函数(内部使用，外部禁止调用)*/
    int init();
    int GetImgRaw();
    void run();
public:

    /** 相机初始化函数(不包括相机参数初始化)
     * @param:const char * device,相机设备名
    *  @param: int width,相机分辨率的宽度值。
    *  @param: int height,相机分辨率的高度值。
    *  @param: bool mjpeg,是否设置为MJPEG格式。默认为true。
     *  @return: int ,错误码，0代表无错误。
    */
    int init(std::string videoPath, int width, int height);

    /** 图片获取函数
     *  @param: Mat &img,　Mat类的引用，目标图像存放。
     *  @return: int ,错误码，0代表无错误。
     */
    int getImg(cv::Mat &img);


    /** 相机信息打印函数
     *  @param: void
     *  @return: int ,错误码，0代表无错误。
     *  @note:  包括相机支持的功能，以及当前分辨率，格式，FPS.
     */
   // int infoPrint();

    /** 检测相机是否打开函数
     *  @param: void
     *  @return: bool ,ture代表正常打开，false代表未打开或异常关闭
     */
    bool isOpen();


    /** 相机格式改变函数
     *  @param: int width,相机分辨率的宽度值。
     *  @param: int height,相机分辨率的高度值。
     *  @param: bool mjpeg,是否设置为MJPEG格式。默认为true。
     *  @return: int ,错误码，0代表无错误。
     */
    int changeVideoFormat(int width, int height);
    void setCaptureState(bool isCapture);
};

#endif //LIB_USB_CAPTURE_WITH_OPENCV_H
