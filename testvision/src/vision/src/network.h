#ifndef NETWORK_H
#define NETWORK_H

#include<iostream>
#include<opencv2/opencv.hpp>
#include"darknet.h"
#include"improcess.h"
 
using namespace std;
using namespace cv;
 
float colors[6][3] = { {1,0,1}, {0,0,1},{0,1,1},{0,1,0},{1,1,0},{1,0,0} };
float get_color(int c, int x, int max);
void getPredictions(Mat& framex,vector<Rect>& boxesx,network* netx,vector<string>& classNamesVecx,vector<int>& classNamesx);

#endif
