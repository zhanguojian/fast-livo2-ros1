#include "vio.h"
#include <vector>

VIOManager::VIOManager()
{
    //定义空函数，实际的初始化在initializeVIO
}

VIOManager::~VIOManager()
{
    delete visual_submap;
    for (auto &pair : warp_map) delete pair.second;
    warp_map.clear();
    for (auto &pair : feat_map) delete pair.second;
    feat_map.clear();
}

//获得 IMU 到激光雷达的外参  ，使用p_imu = rot * p_lidar + transl逆推
void VIOManager::setImuToLidarExtrinsic(const V3D &transl, const M3D &rot)
{
    Pli = -rot.transpose() * tranls;

    //旋转矩阵的逆等于转置
    Rli = rot.transpose() ;
}

//获得激光雷达到相机的外参
void VIOManager::setLidarToCameraExtrinsic(Vector<double> &R, Vector<double> &P)
{
    Rcl << MAT_FROM_ARRAY(R);
    Pcl << VEC_FROM_ARRAY(P);
}

//
void VIOManager:initializeVIO()
{
    //视觉子地图，存储当前帧的视觉跟踪点点云数据容器
    visual_sub_map = new SubSparseMap;

    //相机内参赋值
    fx = cam->fx();
    fy = cam->fy();
    cx = cam->cx();
    cy = cam->cy();

    //图像缩放因子
    image_resize_factor = cam->scale();

    print("intrinsic : %.6lf, %.6lf, %.6lf, %.6lf\n", fx, fy, cx, cy);

    //图像高度和宽度
    width = cam->width();
    height = cam->height();

    print("width:%d, height:%d, scale: %f\n", width, height, image_resize_factor);

    Rci = Rcl * Rli;
    Pci = Rcl * pli + Pcl;

    //计算雅可比矩阵中的旋转和平移相关的部分
    V3D Pic;
    M3D tmp;
    Jdphi_dR = Rci;  //IMU 坐标系下的姿态扰动，经过 Rci 传递到相机坐标系。使用Rci作为旋转部分的雅可比矩阵

    //Camera 原点在 IMU 坐标系下的位置。
    Pic = -Rci.transpose() * Pci;

    //反对称矩阵
    tmp << SKEW_SYM_MATRX(pic);

    //表示旋转到平移的影响
    Jdp_dR = -Rci * tmp;


    //grid_size 是网格像素大小
    if(grid_size > 10)
    {
        //网格数量
        grid_n_width = ceil(static_cast<double>(width / grid_size));
        grid_n_height = ceil(static_cast<double>(height / grid_size));
    }
    else //小于10则用设定好的height和height_n 求 grid_size
    {
       grid_size = static_cast<int>(height / grid_n_height);
       grid_n_height = ceil(static_cast<double>(height / grid_size));
       grid_n_width = ceil(static_cast<double>(width / grid_size));
    }

    //总网格数
    length = grid_n_width * grid_n_height;


    //光线投射
    if(raycast_en)
    {
        //边界标识，标记哪些网格在图像边缘，初始化为0即所有网格都不在边缘
        border_flag.resize(length,0);

        //清空并重新分配rays_with_sample_points，用于存储每个网格的射线和采样点
        std::vector<std::vector<V3D>>().swap(rays_with_sample_points);
        rays_with_sample_points.reserve(length);

        printf("grid_size:%d, grid_n_heignt:%d, grid_n_width:%d,length:%d\n", grid_size, grid_n_height, grid_n_width, length);

        float d_min = 0.1;  //射线采样最小距离
        float d_max = 3.0;  //射线采样最大距离
        float step = 0.2;  //采样间距

        //遍历网格
        for(int grid_row = 1,; grid_row <= grid_n_width; grid_row++)
        {
            for(int grid_col = 1; grid_col <= grid_n_width; grid_col++)
            {
                std::vector<V3D> SamplePointsEachGrid;
                int index = (grid_row - 1) * grid_n_width + grid_col - 1;

                //网格边缘标记
                if(grid_row == 1 || grid_col == 1 || grid_row == grid_n_height || grid_col == grid_n_width)
                {
                    border_flag[index] = 1;
                }

                int u = grid_size / 2 + (grid_col - 1) * grid_size;
                int v = grid_size / 2 + (grid_row - 1) * grid_size;

                for(float d_temp = d_min; d_temp <= d_max; d_temp += step)
                {
                    V3D xyz;
                    //把像素点 (u, v) 反投影成相机坐标系下的一条射线方向
                    xyz = cam->cam2world(u,v);

                    //把这个方向向量缩放到指定深度 d_temp
                    xyz *= d_temp / xyz(2);
                    SamplePointsEachGrid.push_back(xyz);
                }
                //
                rays_with_sample_points[index].push_back(SamplePointsEachGrid);
            }
        }

        //pcd保存，这里是使能开关
        if(colmap_output_en)
        {
            //加载相机模型，把通用相机模型 cam 转成 pinhole_camera
            pinhole_cam = dynamic_cast<vk::pinhole_camera*>(cam);

            //打开 COLMAP 的 images.txt
            fout_colmap.open(DEBUG_FILE_DIR("Colmap/sparse/0/images/txt"),ios::out);

            //写入 images.txt 文件头
            fout_colmap << "# Image list with two lines of data per iamge:\n";
            fout_colmap << "# IMAGE_ID , QW, QX, QY, QZ, TX, TY, TZ, CANERA_ID, NAME\n";
            fout_colmap << "# POINTS2D[] as (X, Y, POINT3D_ID)\n";

            //打开 COLMAP 的 cameras.txt
            fout_camera.open(DEBUG_FILE_DIR("Colmap/sparse/0/cameras.txt"), ios::out);

            //写入相机内参
            fout_camera << "# camera list with one line of data per camera:\n";
            fout_camera << "#  CAMERA_ID, MODLE, WIDTH, HEIGHT, PARAMS[]\n";
            fout_camera << "1 PINHOLE " << width << " " height <<" "
                  << std::fixed << std::setprecision(6)
                  << fx << " " << fy << " "
                  << cx << " " << cy << " " << std::endl;

            //关闭 cameras.txt
            fout_camera.close();
        }

        //图像网格缓存初始化
        grid_num.resize(length);   //一般用于记录每个网格里有多少个视觉点/地图点。
        map_index.resize(length);   //通常用于记录每个网格对应的地图点索引。
        map_dist.resize(length);   //通常记录每个网格中当前选择的地图点距离相机的距离。
        update_flag.resize(length);   //用于标记每个网格或点是否已经被更新/处理。
        scan_value.resize(length);    //一般是视觉点筛选或 raycast 时的辅助分数。

        //总像素
        patch_size_total = patch_size * patch_size;

        //边长一半
        patch_size_half = static_cast<int>(patch_size / 2);

        //给patch的缓存空间
        patch_buffer.resize(patch_size_total);

        //warp patch 相关缓存长度
        warp_len = patch_size_total * patch_pyrimid_level;

        //计算图像边界安全距离
        border = (patch_size_half + 1) * (1 << patch_pyrimid_level);

        //reserve(length)：只预留容量，size 仍然是 0
        retrieve_voxel_points.reserve(length);
        append_voxel_points.reserve(length);

        //只清空不释放容量
        sub_feat_map.clear();
    }

}

//重置，清除网格  std::fill(begin, end, value);把一个范围内的所有元素都设置成同一个值。
void VIOManager::resetgrid()
{
    //这里把所有网格标记成未知状态
    fill(grid_num.begin(), grid_num.end(), TYPE_UNKNOWN);

    //把每个网格对应的地图点索引重置为 0。
    fill(map_index.begin(), map_index.end(), 0);

    //  每个网格的距离初始化成一个很大的值。
    fill(map_dist.begin(), map_dist.end(), 10000.0f);

    //表示所有网格/点当前都没有被更新。
    fill(update_flag.begin(), update_flag.end(), 0);

    //清空每个网格的临时评分或扫描值。
    fill(scan_value.begin(), scan_value.end(), 0.0f);

    //清空并调整检索点容器
    retrieve_voxel_points.clear();
    retrieve_voxel_points.resize(length);

    //清空并调整新增点容器
    append_voxel_points.clear();
    append_voxel_points.resize(length);

    total_points = 0;
}


//
void VIOManager::computeProjectionJacobian(V3D p, MD(2,3) &J)
{
    const double x = p[0];
    const double y = p[1];
    const double z_inv = 1. / p[2];
    const double z_inv_2 = z_inv_ * z_inv;

    J(0,0) = fx * z_inv;
    J(0,1) = 0.0;
    J(0,2) = -fx * x * z_inv_2;
    J(1,0) = 0.0;
    J(1,1) = fy * z_inv;
    J(1,2) = -fy * y * z_inv_2;

}

void VIOManager::getImagePatch(cv::Mat img, V2D pc, float *patch_tmp. int level)