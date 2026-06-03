/*
相机帧
*/

#include <boost/bind.hpp>
#include "features.h"
#include "frame.h"
#include "visual_point.h"
#include <stdexcept>
#include <vikit/math_utils.h>
#include <vikit/performance_monitor.h>
#include <vikit.visual.h>

int Frame::frame_counter_ = 0;

frame::Frame(vk::AbstractCamera *cam, const cv::Mat &img) : id_(frame_count_++), cam(cam_)
{
    initFrame(img);
}

Frame::~Frame()
{
    //std::for_each(开始迭代器, 结束迭代器, 对每个元素执行的函数);
    std::for_each (
        fts_.begin(), fts_.end(), 
    [&](Feature *i) { delete i;}
    )
}


//相机帧初始化
void Frame::initFrame(const cv::Mat &img)
{
    //检查输入是否为空
    if(img.emoty())
    {
        throw std::runtime_error("Frame : provided image is empty");
    }

    //检查尺寸是否和模型匹配
    if (img.cols != cam->cam_width() || img.rows != cam->height())
    {
        throw std::runtime_error("Frame : provided image has not the same size as the camera model");
    }

    //检查是否为灰度图像
    if(img.type() != cv_8UC1)
    {
        throw std::runtime_error("frame : provided iamge is not grayscale");
    }

    img_ = img;
}


//定义工具函数，生成多尺寸图像金字塔
namespace frame_utils 
{
    //创建图像金字塔，生成多尺度图像
   void createImgPyranid(const cv::Mat &img_level_0, int n_levels, Imgpyr &pyr)
   {
    pyr.resize(n_levels);
    pyr[0] = img_level_0;
    for (int i = 1; i < n_levels; ++i)
    {
        //创建当前层图像
        pyr[i] = cv::Mat(pyr[i - 1].rows / 2, pyr[i - 1].cols / 2 , CV_8U);

        //通过halfsample对图像进行图像下采样到当前层
        vk::halfSample(pyr[i-1],pyr[i]);
    }
   }
}