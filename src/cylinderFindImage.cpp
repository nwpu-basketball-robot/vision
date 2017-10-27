#include "cylinderFindImage.h"
//*****************************************************************
#include<iostream>
using namespace std;
//***************************************************************
FindCylinder::FindCylinder()//初始化
{
    //init
    init();
}


FindCylinder::~FindCylinder()
{
}


void FindCylinder::Filter(const cv::Mat& img, cv::Mat& imgThresholded)
{
    //较大程度的平滑图像，减少噪声
    cv::Mat filter1;
    cv::Mat kernal1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(iK1Size, iK1Size));
    cv::morphologyEx(img, filter1, cv::MORPH_OPEN, kernal1);
    cv::morphologyEx(filter1, filter1, cv::MORPH_CLOSE, kernal1);

    //for debug
    //cv::imshow("filter_first", filter1);
    //

    //HSV
    cv::Mat imgHSV;
    cv::cvtColor(filter1, imgHSV, CV_BGR2HSV);
    cv::inRange(imgHSV, cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

    //for debug
    //cv::imshow("inRange", imgThresholded);
    //

    //处理二值图像的噪声
    cv::Mat kernal2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(iK2Size, iK2Size));
    cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, kernal2);
    cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, kernal2);

    //for debug
    //cv::imshow("filter_second", imgThresholded);
    //
}



void FindCylinder::findCy(const cv::Mat& imgThresholded, Cylinder& cy)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i>               hierarchy;
    cv::findContours(imgThresholded, contours, hierarchy,
                     CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    //计算轮廓的矩， 选择拥有最大面积的轮廓
    double maxArea           = -1;
    int    maxAreaIndex      = -1;
    std::vector<cv::Moments> mu(contours.size());
    for(size_t i=0; i<contours.size(); i++)
    {
        cv::Rect rt      = cv::boundingRect(contours[i]);
        double   numbers = rt.height * rt.width;
        mu[i]            = cv::moments(contours[i], false);

        //第一种情况 矩阵各方面符合要求
        if((mu[i].m00 > maxArea) && ((rt.height + 0.0) / rt.width) > rect_hw_rate_  && (numbers > rect_area_))
        {
            maxArea      = mu[i].m00;
            maxAreaIndex = i;
        }
        //第二种情况 矩阵的面积很大
        else if((mu[i].m00 > maxArea) && (numbers > 3*rect_area_) && ((rt.height + 0.0) / rt.width) > 1.5)
        {
            maxArea      = mu[i].m00;
            maxAreaIndex = i;
        }
    }


    if(maxAreaIndex == -1)
    {
        cy.confidence = false;
    }
    else
    {
        cy.confidence = true;
        cy.boundRect = cv::boundingRect(contours[maxAreaIndex]);
        cy.center.x  = cy.boundRect.x + cy.boundRect.width / 2;
        cy.center.y  = cy.boundRect.y + cy.boundRect.height / 2;
        cy.mu        = mu[maxAreaIndex];
   }
}


//基本思路：
//1，利用HSV，阈值化图像
//2，通过阈值化图像，查找拥有最大面积的轮廓
//3，通过包围轮廓的最小矩形，利用 高度和宽度比 和 面积阈值,筛选

bool FindCylinder::run(const cv::Mat   & image,
                             cv::Point & center,
                             cv::Rect  & rect)
{
    cv::Mat          imgOriginal = image.clone();

    //改变图像的尺寸
    cv::resize(imgOriginal,imgOriginal,cv::Size(imgOriginal.cols/iZoomSize,imgOriginal.rows/iZoomSize));

    //二值化图像
    cv::Mat imgThresholded;
    Filter(imgOriginal, imgThresholded);

    //查找拥有最大面积的轮廓
    Cylinder cy;
    findCy(imgThresholded, cy);
    
    
    //对于不可信的矩阵输出 点的坐标为 （-1，,1）
    if(cy.confidence == false){
            center.x = -1;
            center.y = -1;

            return false;
    }
    else{
        center.x = static_cast<int>(cy.center.x) * iZoomSize;
        center.y = static_cast<int>(cy.center.y) * iZoomSize;

        rect = cy.boundRect;
        rect.x = static_cast<int>(rect.x) * iZoomSize;
        rect.y = static_cast<int>(rect.y) * iZoomSize;
        rect.width = static_cast<int>(rect.width) * iZoomSize;
        rect.height = static_cast<int>(rect.height) * iZoomSize;

        //for debug
        /*
        printf("image find result rect area 　　　　　　　: %d\n",     cy.boundRect.height * cy.boundRect.width);
        printf("image find result rect height/width rate　: %.3lf\n",  (cy.boundRect.height + 0.0) / (cy.boundRect.width));
        */
        //
        return true;
    }
}


//更换场景时，更换参数
void FindCylinder::init()
{
    //定义h，s，v的阈值
    iLowH = 20;     //20
    iHighH = 144;

    iLowS = 103;   //103
    iHighS = 239;  //255

    iLowV = 80;    //80
    iHighV = 255;  //255


    //定义滤波模块核的大小
    iK1Size  = 6;
    iK2Size  = 5;

    //定义图像缩放
    iZoomSize = 1;

    //矩形的筛选面积
    rect_area_ =    2500;
    rect_hw_rate_ = 2.00;

    /*
    //for debug
    cv::namedWindow("filter_second", cv::WINDOW_NORMAL);
    //
    */
}

void FindCylinder::setParameters(const int & ilowh,
                                 const int & ihighh,
                                 const int & ilows,
                                 const int & ihighs,
                                 const int & ilowv,
                                 const int & ihighv,
                                 const int & rectarea)
{
    //定义h，s，v的阈值
    iLowH      = ilowh;     //20
    iHighH     = ihighh;

    iLowS      = ilows;   //103
    iHighS     = ihighs;  //255

    iLowV      = ilowv;    //80
    iHighV     = ihighv;  //255

    rect_area_ = rectarea;
}

