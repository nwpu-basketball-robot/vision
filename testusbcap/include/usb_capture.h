////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      UsbCapture Code for robot
///ALL RIGHTS RESERVED
///@file:usb_capture.h
///@brief: 基于v4l2的封装相机操作(ov2710相机)
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 17-12-20
///修订历史：
///2017.12.29 ：解决帧延迟，，即获取的图片为最新的。可实时调整分辨率，曝光时间参数。(参考2016DJI开源代码)
///2018.3.25：可实时调整相机参数：分辨率，曝光时间等。（新增部分参数调整接口）
/// 增加相机相关参数接口，如相机内参，相机畸变参数，相机云台坐标系偏移量，相机枪管安装偏移量。(新增)
///2018.3.26:相机参数采用yaml配置文件进行读取。完成该版本的开发。
///2018.5.1:增加相机号遍历重连方法，解决usb突然松动导致相机号变动问题。（只适应只有一个相机的情况
/// 在getImg方法中失败时调用）
///2018.5.17:取消机相关参数接口（由于用处不大，直接读取配置文件就可以了），
/// 取消相机重连接口（udev固定设备路径，图片获取失败，重新初始化即可）
//////////////////////////////////////////////////////////////////////////////////


#ifndef LIB_USB_CAPTURE_H
#define LIB_USB_CAPTURE_H

#include <opencv2/opencv.hpp>



class UsbCapture{
public:
    UsbCapture();
    ~UsbCapture();
private:
    std::string mVideoPath;
    int mVideofd;//设备文件描述符
    bool mIsInit;
    bool mIsOpen;
    uint8_t *mBuffer;//摄像头原始缓存数据
    int mBufferLen;
    int mCaptureWidth;
    int mCaptureHeight;
    bool mIsMjpeg;
    unsigned int mImgFormat;//获取图像格式，和摄像头相关

private:
    int setVideoFormat(int width, int height, bool MJPEG=true);
    int init_mmap();
    void cvtRaw2Mat(const void * data, cv::Mat & image);
    int refreshVideoFormat();
    /** 相机初始化函数(内部使用，外部禁止调用)*/
    int init();
public:
    /** 相机初始化函数
     * @param:const char * device,相机设备名
    *  @param: int width,相机分辨率的宽度值。
    *  @param: int height,相机分辨率的高度值。
    *  @param: bool mjpeg,是否设置为MJPEG格式。默认为true。
     *  @return: int ,错误码，0代表无错误。
    */
    int init(std::string videoPath,int width,int height,bool mjpeg=true);

    /** 图片获取函数
     *  @param: Mat &img,　Mat类的引用，目标图像存放。
     *  @return: int ,错误码，0代表无错误。
     */
    int getImg(cv::Mat &img);

    /** 图片获取操作符>>重载
     *  @param: Mat &img,　Mat类的引用，目标图像存放。
     *  @note:  不建议采用该方式，因为没有返回值，可能采集失败，而不知道，引起程序崩溃。
     */
    UsbCapture& operator >> (cv::Mat & img);

    /** 相机信息打印函数
     *  @param: void
     *  @return: int ,错误码，0代表无错误。
     *  @note:  包括相机支持的功能，以及当前分辨率，格式，FPS.
     */
    int infoPrint();

    /** 检测相机是否打开函数
     *  @param: void
     *  @return: bool ,ture代表正常打开，false代表未打开或异常关闭
     */
    bool isOpen();

    /** 相机曝光设置函数
     *  @param: int t,手动曝光模式的曝光时间设置。
     *  @return: int ,错误码，0代表无错误。
     *  @note:  如果auto_exp为true，为自动曝光模式，则第二个参数t无效。
     */
    int setExposureTime(int t);

    //设置色彩饱和度0-128
    int setSaturation(int value);
    //设置白平衡
    int setWhiteBalance(int value);
    //设置亮度
    int setBrightness(int value);
    //设置伽玛值
    int setGamma(int value);
    //设置对比度
    int setContrast(int value);
    /** 相机格式改变函数
     *  @param: int width,相机分辨率的宽度值。
     *  @param: int height,相机分辨率的高度值。
     *  @param: bool mjpeg,是否设置为MJPEG格式。默认为true。
     *  @return: int ,错误码，0代表无错误。
     */
    int changeVideoFormat(int width, int height, bool mjpeg=true);

};







#endif //LIB_USB_CAPTURE_H
