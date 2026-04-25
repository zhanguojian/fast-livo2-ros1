/*
流型空间到流式空间的变换，群的指数映射和对数映射，
*/

#pragma once
#include <Eigen/Core>
#include <cmath>
#include <math.h>

//SO(3)群的指数映射，输入为一个3维向量，输出为一个3x3的旋转矩阵
//把三维向量V写成对应的反对称矩阵
#define SKEW_SYM_MATRX(v) 0.0, -v(2), v(1), \
                          v(2), 0.0, -v(0), \
                          -v(1), v(0), 0.0

template <typename T> Eigen::Matrix<T,3,3> Exp(const Eigen::Matrix<T,3,1> &ang)
{
    T ang_norm = ang.norm();
    Eigen::Matrix<T,3,3> Eye3 = Eigen::Matrix<T,3,3>::Identity();

    if(ang_norm > 0.0000001)
    {
        Eigen::Matrix<T,3,3> r_axis = ang / ang_norm ;
        Eigen::Matrix<T,3,3> K;
        K << SKEW_SYM_MATRX(r_axis);

        return Eye3 + std::sin(ang_norm) * K + std::cos(ang_norm) * K * K;
    }
    else {
        return Eye3;
    }
}

template <typename T,typename Ts> Eigen::Matrix<T,3,3> Exp(const Eigen::Matrix<T,3,1> &ang_vel, const T &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T,3,3> Eye3 = Eigen::Matrix<T,3,3>::Identity();

    if( ang_vel_norm > 0.00000001)
    {
        Eigen::Matrix<T,3,1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T,3,3> K;
        K << SKEW_SYM_MATRX(r_axis);

        T r_ang = ang_vel_norm * dt; 

        return Eye3 + std::sin(r_ang) * K + std::cos(r_ang) * K * K ;
    }
    else {
        Eye3;
    }
}

template <typename T> Eigen::Matrix<T,3,3> Exp(const T &v1, const T &v2, const T &v3)
{
    //&&是C++11引入的右值引用，表示v1、v2、v3可以是临时对象或者将要被移动的对象，这样可以避免不必要的复制，提高性能。
    T &&norm = sqrt(v1*v1 + v2*v2 + v3*v3);
    Eigen::Matrix<T,3,3> Eye3 = Eigen::Matrix<T,3,3>::Identity();

    if(norm > 0.00000001)
    {
        T r_ang[3] = {v1/norm, v2/norm, v3/norm};
        Eigen::Matrix<T,3,3> K;
        K << SKEW_SYM_MATRX(r_ang);

        return Eye3 + std::sin(norm) * K + std::cos(norm) * K * K ;
    }
    else {
        return Eye3;
    }
}

//SO(3)群的对数映射，输入为一个3x3的旋转矩阵，输出为一个3维向量
template <typename T> Eigen::Matrix<T,3,1> Log(const Eigen::Matrix<T,3,3> &R)
{
    //求旋转角度，R.trace()是旋转矩阵的迹，即对角线元素之和，3.0是单位矩阵的迹，1e-6是一个很小的数，用于避免数值不稳定。
   T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : acos((R.trace() - 1.0) / 2.0);
   Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));

   return (std::abs(theta) < 1e-6) ? K / 2.0 : theta * K / (2.0 * sin(theta));

}

//暂时不理解，后面研究
//旋转矩阵转欧拉角，输入为一个3x3的旋转矩阵，输出为一个3维向量，分别表示绕x、y、z轴的旋转角度
template<typename T > Eigen::Matrix<T,3,1> RotMtoEuler(const Eigen::Matrix<T,3,3> &rot)
{
    T sy = sqrt(rot(0,0) * rot(0,0) + rot(1,0) * rot(1,0));
    bool singular = sy < 1e-6; // 如果sy很小，说明旋转矩阵接近于奇异状态
    T x, y, z;
    if (!singular)
    {
        x = atan2(rot(2,1), rot(2,2));
        y = atan2(-rot(2,0), sy);
        z = atan2(rot(1,0), rot(0,0));
    }
    else
    {
        x = atan2(-rot(1,2), rot(1,1));
        y = atan2(-rot(2,0), sy);
        z = 0;
    }
    Eigen::Matrix<T,3,1> ang(x, y, z);
    return ang;
}