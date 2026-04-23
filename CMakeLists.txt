cmake_minimum_required(VERSION 3.0)
project(rewriter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set (CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
//prethread 链接 POSIX 线程库  fexceptions启用异常处理并生成传播异常所需的额外代码,默认开启
set(CMAKE_CXX_FLAGS "-pthread -fexceptions")  # 这是所有编译编译模式都使用的编译选项

message(STATUS "current cpu architecture: ${CMAKE_SYSTEM_PROCESSOR}")



# -O3 优化级别， -O3 是最高的优化级别，启用所有优化选项以提高性能 -O0 是最低的优化级别，禁用所有优化选项以便于调试 -O1 和 -O2 是中等的优化级别，启用一些优化选项以提高性能，但不会像 -O3 那样激进

# -mcpu=native 生成针对本地CPU的代码,编译器会自动检测当前系统的CPU类型，并生成针对该CPU的优化代码  cpu级别

# -march=native 生成针对不同CPU架构的代码，拿到不同的CPU架构会生成不同的代码  架构级别

# -mtune=native 优化代码以在本地机器上运行， 优化级别

# -funroll-loops 循环展开优化，减少循环控制的开销，适用于循环迭代次数较少且循环体较小的情况

# -ffast-math 启用快速数学运算，可能会牺牲一些精度，但可以提高性能，不适合高精度计算的场景


if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)$")
 if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|ARM)$")

    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Release build flags: ${CMAKE_CXX_FLAGS_RELEASE}")

  elseif()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Release build flags: ${CMAKE_CXX_FLAGS_RELEASE}")
  
  endif()

elseif()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops ")
    message(STATUS "Release build flags: ${CMAKE_CXX_FLAGS_RELEASE}")
    add_definitions(-DX86_ARCH) // 定义宏以区分不同架构

endif()

#-D相当与#define  -DROOT_DIR=\"${CMAKE_SOURCE_DIR}/\"  定义一个宏ROOT_DIR，值为当前CMake项目的源代码目录路径，路径以斜杠结尾以便与其他路径拼接使用
# \" 转义双引号，用于在宏定义中包含双引号，使得宏值成为一个字符串字面量，

add_definitions(-DROOT_DIR=\"${CMAKE_SOURCE_DIR}/\")

include(ProcessorCount)  # 引入ProcessorCount模块，用于获取处理器数量
ProcessorCount(N)            # 获取处理器数量并存储在变量N中
message(STATUS "Number of processors: ${N}")  # 输出处理器数量

if(N GREATER 4)
  match(EXPR PROC_NUM "4" )  //EXPR 后面内容当数学表达式计算，相当于set(PROC_NUM 4) 
  add_definitions(-DMP_EN -DMP_DPROC_NUM=${PROC_NUM})  // 定义宏MP_EN启用多线程处理，并定义宏MP_DPROC_NUM设置多线程处理的线程数量为4
elseif(N GREATER 1)
    match(EXPR PROC_NUM "${N}" )  // 将处理器数量赋值给PROC_NUM
    add_definitions(-DMP_EN -DMP_DPROC_NUM=${PROC_NUM})
else()
    add_definitions(-DMP_DPROC_NUM=1)  # 定义宏MP_DPROC_NUM设置多线程处理的线程数量为1，即单线程处理
endif() 

find_package(OpenMP QUIET)  #查找OpenMP库，如果找到则设置OpenMP_FOUND为TRUE
if(OpenMP_CXX_FOUND)        #如果找到OpenMP库
    message(STATUS "OpenMP found, enabling OpenMP support")  #输出找到OpenMP库的消息
    add_compile_options(${OpenMP_CXX_FLAGS})   #将OpenMP的编译标志添加到CMAKE_CXX_FLAGS中
else()
    message(STATUS "OpenMP not found, proceeding without OpenMP support")  #输出未找到OpenMP库的消息
endif()

find_package(mimalloc QUIET)
if(mimalloc_FOUND)
    message(STATUS "mimalloc found, enabling mimalloc support")
else()
    message(STATUS "mimalloc not found, proceeding without mimalloc support")  
endif()

# 查找catkin构建系统，因为这些倚赖是通过catkin构建的，REQUIRED表示如果没有找到该包则会报错并停止构建，COMPONENTS后面跟着需要的catkin包列表，这些包提供了必要的功能和接口供当前项目使用
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    sensor_msgs
    roscpp
    rospy
    std_msgs
    tf
    livox_ros_driver
    message_generation
    eigen_ conversions
    vikit_commom
    vikin_ros
    cv_bridge
    image_transport    
    )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(boost REQUIRED COMPONENTS thread)

#catkin_package函数用于定义当前包的属性和依赖关系，CATKIN_DEPENDS后面跟着当前包依赖的其他catkin包列表，这些包提供了必要的功能和接口供当前包使用，DEPENDS后面跟着当前包依赖的非catkin库列表，这些库提供了必要的功能和接口供当前包使用
#简单来说是别人调用这个包时需要知道这个包依赖哪些其他的包和库，以便正确地链接和使用这些依赖项
catkin_package(  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    std_msgs
    message_runtime

    roscpp
    rospy

    vikit_commom
    vikin_ros
    cv_bridge
    image_transport    
    DEPENDS EIGENS PCL OpenCV Sophus
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS} #这是find_package(catkin ...)命令找到的catkin包的头文件路径，这些路径包含了当前包依赖的catkin包的头文件，可以在当前包的源代码中使用这些头文件提供的功能和接口
    ${OpenCV_INCLUDE_DIRS} #这是find_package(OpenCV ...)命令找到的OpenCV库的头文件路径，这些路径包含了OpenCV库的头文件，可以在当前包的源代码中使用这些头文件提供的功能和接口
    ${EIGEN3_INCLUDE_DIRS} #这是find_package(Eigen3 ...)命令找到的Eigen库的头文件路径，这些路径包含了Eigen库的头文件，可以在当前包的源代码中使用这些头文件提供的功能和接口
    ${Sophus_INCLUDE_DIRS} #这是find_package(Sophus ...)命令找到的Sophus库的头文件路径，这些路径包含了Sophus库的头文件，可以在当前包的源代码中使用这些头文件提供的功能和接口
    ${PCL_INCLUDE_DIRS}   #这是find_package(PCL ...)命令找到的PCL库的头文件路径，这些路径包含了PCL库的头文件，可以在当前包的源代码中使用这些头文件提供的功能和接口
)

add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_pro src/imu_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

add_executable(fastlivo_mapping src/main.cpp)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)  # 设置可执行文件的输出路径为项目源代码目录下的bin文件夹


target_link_libraries(fastlivo_mapping
    vio
    lio
    pre
    imu_pro
    laser_mapping
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
)

if(mimalloc_FOUND)
    target_link_libraries(fastlivo_mapping mimalloc)  # 将mimalloc库链接到可执行文件
endif()