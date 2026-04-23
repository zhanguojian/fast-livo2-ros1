/*
流型空间到流式空间的变换，群的指数映射和对数映射，
*/

#pragma once
#include <Eigen/Core>
#include <math.h>

//SO(3)群的指数映射，输入为一个3维向量，输出为一个3x3的旋转矩阵
//把三维向量V写成对应的反对称矩阵
#define SKEW_SYM_MATRX(v) 0.0, -v(2), v(1), \
                          v(2), 0.0, -v(0), \
                          -v(1), v(0), 0.0
