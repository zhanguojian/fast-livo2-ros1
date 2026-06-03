/*
视觉点
*/

#pragma once

#include <boost/noncopyable.hpp>    //不允许拷贝
#include "common_lib.h"
#include "frame.h"


//前向声明，告诉编译器Feature类的存在，但不提供其定义，这样可以避免在头文件中包含Feature类的完整定义，从而减少编译时间和依赖关系。
class Feature;

class VisualPoint : public boost::noncopyable     //视觉点不允许被拷贝
{
public:

    //内存对齐，Eigen库要求在使用Eigen类型的成员变量时，类必须使用EIGEN_MAKE_ALIGNED_OPERATOR_NEW宏来确保内存对齐，以避免潜在的性能问题和错误。
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    //三维地图点在世界坐标系下的位置
    Vector3d pos_;

    //这个视觉点在局部表面的法向量
    Vector3d normal_;

    //法向量的信息矩阵
    Matrix3d normal_information_;

    //上一次的法向量
    Vector3d previous_normal_;

    //视觉点在图像中被观测到的特征列表，Feature类表示在图像中检测到的特征点，这些特征点与这个视觉点相关联，obs_是一个指向Feature对象的指针列表，表示这个视觉点在不同图像帧中被观测到的特征点集合。
    list<Feature*> obs_;

    //视觉点的协方差矩阵，表示这个视觉点位置的不确定性
    Eigen::Matrix3d covariance_;

    //视觉点是否已经收敛，收敛意味着这个视觉点的位置已经稳定，不再发生显著变化。
    bool is_converged_;

    //判断法向量是否已经初始化，判断是否可用
    bool is_normal_initialized_;

    //判断该视觉点是否有一个参考特征点，ref_patch_是一个指向Feature对象的指针，表示这个视觉点的参考特征点
    bool has_ref_patch_;

    //参考patch对应的特征观测
    Feature *ref_patch_;

    //构造函数和析构函数，创建视觉地图点时传入初始三维位置：
    VisualPoint(const Vector3d &pos);
    ~VisualPoint();

    //寻找与当前视觉点位置最接近的特征点，并将其指针赋值给ftr，返回值表示是否成功找到一个特征。
    void findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const;

    //作用：从 obs_ 中找到一个最合适的 Feature，赋给 ftr。
    void deleteNonRefPatchFeatures();

    //删除不是参考 patch 的观测。
    void deleteFeatureRef(Feature *ftr);

    //从 obs_ 里删除某一个指定的观测 ftr。
    void addFrameRef(Feature *ftr);

    //从已有观测中找一个和当前视角比较接近的观测。当前帧看到这个点，需要从历史观测里找一个视角接近的 Feature，用于 patch 对齐或光度匹配
    bool getCloseViewobs(const Vector &pos, Feature *&obs, const Vector2d &cur_px) const;

}


