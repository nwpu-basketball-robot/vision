#ifndef CYLINDER_FIND_H
#define CYLINDER_FIND_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>

//轮廓信息
struct Cylinder
{
    cv::Moments mu;     //保存原始轮廓信息
    cv::Rect boundRect; //包围轮廓的最小正立矩阵
    cv::Point2f center; //矩阵中心

    bool confidence; //可信度 可信度false时 ,不含有信息
};

class FindCylinder
{
  private:
    //HSV Threshold
    int iLowH;
    int iHighH;

    int iLowS;
    int iHighS;

    int iLowV;
    int iHighV;

    //滤波模块的核大小
    int iK1Size;
    int iK2Size;

    //图像的缩放尺寸
    int iZoomSize;

    //矩形的筛选面积
    int rect_area_;

    //矩形的长宽比
    double rect_hw_rate_;

  private:
    //初始化
    void init();

    //二值化图像
    void Filter(const cv::Mat &img, cv::Mat &imgThresholded);

    //利用二值化图像，查找最大轮廓的最小包围矩形
    void findCy(const cv::Mat &imgThresholded, Cylinder &cy);

  public:
    FindCylinder();
    ~FindCylinder();

    //针对每一幅图像操作
    bool run(const cv::Mat &image,
             cv::Point &center,
             cv::Rect &rect);

    //for set Parameters
    void setParameters(const int &ilowh,
                       const int &ihighh,
                       const int &ilows,
                       const int &ihighs,
                       const int &ilowv,
                       const int &ihighv,
                       const int &rectarea);
};

#endif // CYLINDER_FIND_H
