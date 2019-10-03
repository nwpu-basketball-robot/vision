/**
 * @file usb_capture_with_thread.h
 * @brief 介于之前的VideoCapture模块问题大得很，选择了重写，但接口上保持了一致以确保兼容性。
 * 去掉了一些不常用的接口以减(多)少(摸)工(一)作(会)量(儿)
 * @author XiaMo
 * @version v0.1
 * @date 2019-04-21
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <iostream>
#include <mutex>
#include <thread>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace wmj{

class UsbCaptureWithThread{
private:
    std::string _devicePath;
    int _devicefd;
    bool _isInitialized;
    bool _isOpen;
    uint8_t _captureState; // 0是捕捉，1是暂停，2退出主循环，终止线程。

    struct videoBuffer{
        uint8_t *pBuffer;
        uint32_t len;
        int width;
        int height;
    }*_Buffer;

    u_int32_t n_buffers;

    cv::Mat _imgBuf;

    bool _isUpdated;
    std::mutex _dataMutex;
    std::mutex _signMutex;

    std::thread captureThread;

    uint32_t _imageWidth;
    uint32_t _imageHeight;

    int openDevice();

    int closeDevice();

    int initDevice();

    int initBuffers();

    int freeBuffers();

    int startStream();

    int stopStream();

    int getFrameRaw();

    void mainRun();

public:
    UsbCaptureWithThread(std::string, uint32_t, uint32_t);
    UsbCaptureWithThread();
    ~UsbCaptureWithThread();


    /**
     * @brief 为了和CV一致，重载了输入(>>)运算符
     *
     * @param cv::Mat
     *
     * @return 
     */
    int operator >> (cv::Mat&);

    int getImg(cv::Mat&);

    int infoPrint();

    bool isOpen();

    int setExposureTime(int value);

    int setBrightness(int value);

    int setGamma(int value);

    int setContrast(int value);

    int changeVideoFormat(int width, int height);

    void setCaptureState(bool isCapture);
};
}

