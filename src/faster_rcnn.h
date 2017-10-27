#ifndef FASTER_RCNN_HPP
#define FASTER_RCNN_HPP

#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <utility>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <boost/python.hpp>

#include "gpu_nms.hpp"
#include "caffe/caffe.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
 * ===  Class  ======================================================================
 *         Name:  Detector
 *  Description:  FasterRCNN CXX Detector
 * =====================================================================================
 */

class Detector 
{

private:

	//the coco caffe model number
	static const int class_num = 6;

	static const float CONF_THRESH = 0.4;
    static const float NMS_THRESH = 0.3;
    
    static const int  max_input_side = 1000;
    static const int  min_input_side = 600;
    
	//shared_ptr是引用计数指针
    caffe::shared_ptr< caffe::Net<float> >  net_;

private:

	void boxes_sort(int num, const float* pred, float* sorted_pred);
	void choose_boxes( std::vector< cv::Rect > & results, const int* keep, const int & num_out, const float* sorted_pred_cls);
	void bbox_transform_inv(const int num, const float* box_deltas, const float* pred_cls, float* boxes, float* pred, int img_height, int img_width);
    
public:

	Detector()
	{

	}

	~Detector()
	{

	}
    
    Detector(const std::string & model_file, const std::string & weights_file);  
    
    void detect( const cv::Mat & cv_img , std::map<int, std::vector< cv::Rect > > & results);

};

//box sort
struct Info
{
    float score;
    const float* head;
};

bool compare(const Info & Info1, const Info & Info2);

#endif
