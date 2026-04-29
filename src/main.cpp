/*
livo2算法入口
*/

#include "LIVMapper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");    //初始化节点
    ros::NodeHandle nh;                     //ros句柄通信
    image_transport::image_transport it(nh);     //ros图像传输工具类
    LIVMapper mapper(nh);                       //初始化建图器
    mapper::initializeSubscribersAndPublishers(nh, it);      //初始化订阅器和发布器
    mapper.run();                                //系统运行
    return 0;
}