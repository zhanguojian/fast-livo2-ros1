/*
构造函数VisualPoint接收一个三维向量pos，用来初始化pos_，也就是这个点的三维位置。本质上是一个地图点容器
其他成员变量如previous_normal_和normal_初始化为零向量，is_converged_、is_normal_initialized_和has_ref_patch_都初始化为false。
这些变量可能用于记录法线方向、收敛状态、法线是否初始化以及是否有参考图像块（ref_patch）
*/

#include "visual_point.h"
#include "feature.h"
#include <stdexcept>              //这是 C++ 标准库异常头文件，提供了各种异常类的定义，如 std::runtime_error、std::invalid_argument 等，用于在程序中抛出和处理错误。
#include <vikit/math_utils.h>


/*
pos_：3D 空间中的位置坐标
normal_ & previous_normal_：存储法向量信息
is_converged_：该点是否已经收敛（用于优化）
is_normal_initialized_：法向量是否初始化
has_ref_patch_：是否有参考特征点（Patch）
*/
VisualPoint::VisualPoint(const Vector3d &pos) 
    : pos_(pos), previous_normal_(Vector3d::Zero()), normal_(Vector3d::Zero()), is_converged_(false), is_normal_initialized_(false), has_ref_patch_(false)
{
}

//析构函数，负责清理资源，删除与该视觉点相关联的特征观测，并将 obs_ 列表清空，同时将 ref_patch_ 指针置空。
VisualPoint::~VisualPoint()
{
    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
    {
        delete *it;
    }
    obs_.clear();
    ref_patch_ = nullptr;
}


//删除特定的 patch 的观测，如果这个 patch 是当前的参考 patch，那么需要将 has_ref_patch_ 设置为 false，并将 ref_patch_ 指针置空。
void VisualPoint::deleteFeatureRef(Feature *ftr)
{
    //如果要删除的特征点是当前的参考 patch，那么需要更新 has_ref_patch_ 和 ref_patch_ 的状态
    if (ref_patch == ftr)
    {
        ref_patch = nullptr;
        has_ref_patch_ = false;
    }

    //从 obs_ 中删除指定的观测 ftr
    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
    {
        if (*it == ftr)
        {
            delete((*it));
            obs_.erase(it);
            return;
        }
    }

}


//添加观测
void VisualPoint::addFrameRef(Feature *ftr)
{
    //将新的特征点观测 ftr 添加到 obs_ 列表中，表示这个视觉点在当前帧中被观测到了这个特征点。
    obs_.push_back(ftr);
}


//从历史观测中找到当前相机位置framepos视角接近的特征，返回 true 表示成功找到一个合适的特征点，并将其指针赋值给 ftr。
bool VisualPoint::getCloseViewObs(const Vector3d &framepos, Feature *&ftr, const Vector2d &cur_px) const
{
    //如果观测列表为空，直接返回 false，表示没有找到合适的特征点。
    if(obs_.size() <= 0)
    {
        return false;
    };

    //地图点pos_单到当前帧位置framepos的观测方向 obs_dir，计算方法是 framepos - pos_，然后进行归一化处理。
    Vector3d obs_dir(framepos - pos_);
    obs_dir.normalize();

    //定义初始观测点迭代器 min_it，初始值为 obs_ 的 begin()。
    auto min_it = obs_.begin();

    //定义当前一个变量 min_cos_angle 用于记录当前找到的特征点与观测方向的夹角余弦值，初始值为 0
    double min_cos_angle = 0;

    //遍历 obs_ 中的每个特征点，计算它们与当前观测方向 obs_dir 的夹角余弦值，并找到夹角余弦值最大的特征点。
    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
    {
        //计算当前特征点的世界坐标系位置 ((*it)->T_f_w_.inverse().translation())，计算该特征点位置与当前帧位置 framepos 的方向向量 dir，并进行归一化处理。
        Vector3d dir((*it)->T_f_w_.inverse().translation() - pos_);

        //对 dir 进行归一化处理，使其成为一个单位向量，表示从地图点 pos_ 指向当前特征点的方向。
        dir.normalize();

        //计算当前特征点与观测方向 obs_dir 的夹角余弦值 cos_angle，计算方法是 obs_dir.dot(dir)，其中 dot() 是向量的点积运算。
        double cos_angle = obs_dir.dot(dir);
        if (cos_angle > min_cos_angle)
        {
            min_cos_angle = cos_angle;
            min_it = it;
        }
    }
    ftr = *min_it;

    if (min_cos_angle < 0.5) //如果找到的特征点与当前视角的夹角过大，可能不适合用来进行 patch 对齐或光度匹配，因此返回 false。
    {
        return false;
    }

    return true;
}


//遍历所有观测，找到当前得分最小的特征点，将其赋值为ftr参数
void VisualPoint::findMinScoreFeature(const Vector3d &framepos, Feature *ftr) const
{
    //把开始观测初始化设置为最小分数的特征点
    auto min_it = obs_.begin();

    //定义一个变量 min_score 用于记录当前找到的特征点的分数，初始值为一个很大的数（std::numeric_limits<float>::max()），表示还没有找到任何特征点。
    float min_score = std::numeric_limits<float>::max();

    //遍历 obs_ 中的每个特征点，计算它们与当前帧位置 framepos 的分数，并找到分数最小的特征点。
    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
    {
        //当前特征点的分数 ((*it)->score) 小于当前记录的最小分数 min_score 时，更新 min_score 的值，并将 min_it 更新为当前特征点的迭代器 it。
        if ((*it)->score < min_score)
        {
            min_score = (*it)->score;
            min_it = it;
        }
    }
    ftr = *min_it;
}

//从 obs_ 中删除所有不是参考 patch 的观测，这些观测可能是一些不可靠的特征点，因此需要将它们删除以保持视觉点的稳定性和准确性。
void VisualPoint::deleteNonRefPatchFeatures()
{
    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
    {
        if (*it != ref_patch_)
        {
            delete(*it);
            it =  obs_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}