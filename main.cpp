#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;

int main()
{
    //1. 导入点云数据
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZINormal>);
    // 创建输入点云指针
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZINormal>);
    // 创建输出点云指针
    const std::string output_path =
    "/home/lune/radar_homework/build/voxel_filtered.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZINormal>("/home/lune/radar_homework/point_lio_scans.pcd", *cloud_in) == -1)
    {
        std::cerr << "无法读取文件 /home/lune/radar_homework/point_lio_scans.pcd" << std::endl;
        return -1;
    }

    std::cout << "点数: " << cloud_in->size() << std::endl;

     // 2. 打印 XYZ 的最小值和最大值,计算包围盒的长宽高
    pcl::PointXYZINormal min_pt, max_pt;
    pcl::getMinMax3D(*cloud_in, min_pt, max_pt);
    std::cout << "X 范围: [" << min_pt.x << ", " << max_pt.x << "]" << std::endl;
    std::cout << "Y 范围: [" << min_pt.y << ", " << max_pt.y << "]" << std::endl;
    std::cout << "Z 范围: [" << min_pt.z << ", " << max_pt.z << "]" << std::endl;

    std::cout << "包围盒长度: "
          << max_pt.x - min_pt.x << std::endl;

    std::cout << "包围盒宽度: "
          << max_pt.y - min_pt.y << std::endl;

    std::cout << "包围盒高度: "
          << max_pt.z - min_pt.z << std::endl;

      //3. 打印点云信息
      std::cout << "点云的字段信息:" << std::endl;
      
    for (std::size_t i = 0;i < std::min<std::size_t>(5, cloud_in->size());++i)
    {
    const auto& p = cloud_in->points[i];

    cout << "点 " << i
         << ": intensity=" << p.intensity
         << ", normal_x=" << p.normal_x
         << ", normal_y=" << p.normal_y
         << ", normal_z=" << p.normal_z
         << ", curvature=" << p.curvature
         << endl;

    }
    //4. 体素降采样

    // 初始化VoxelGrid滤波器
    pcl::VoxelGrid<pcl::PointXYZINormal> voxel_filter;
    voxel_filter.setInputCloud(cloud_in);           // 设置输入点云
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素边长（单位：米）
    voxel_filter.setDownsampleAllData(false);       // 是否对所有字段（如颜色）降采样
    voxel_filter.setMinimumPointsNumberPerVoxel(1);       // 每个体素至少保留一个点
    voxel_filter.filter(*cloud_filtered);          // 执行滤波

    // 保存降采样后的点云到 PCD 文件
    if (pcl::io::savePCDFileBinary(
      output_path, *cloud_filtered) == -1)
      {
            std::cerr << "保存 PCD 文件失败！" << std::endl;
            return -1;
    }

    std::cout << "点云已保存到 " << output_path << std::endl;

    // 读取保存的点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(output_path, *cloud_filtered) == -1)
    {
        std::cerr << "无法读取文件 " << output_path << std::endl;
        return -1;
    }

    std::cout << "进行体素降采样后的点数: " << cloud_filtered->size() << std::endl;

    double length = max_pt.x - min_pt.x;
    double width  = max_pt.y - min_pt.y;
    double height = max_pt.z - min_pt.z;

    double volume = length * width * height;

    double density_before = cloud_in->size() / volume;
    double density_after  = cloud_filtered->size() / volume;

    std::cout << "包围盒体积: " << volume << std::endl;
    std::cout << "原始平均点密度: "
    << density_before << " 点/m^3" << std::endl;
    std::cout << "降采样后平均点密度: "
    << density_after << " 点/m^3" << std::endl;


    return 0;
}