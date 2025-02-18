#ifndef CUSTOM_POINT_TYPES_H
#define CUSTOM_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>


namespace pcl
{
    struct PointXYZIRGBNormal
    {
        PCL_ADD_POINT4D;         // XYZ座標 + パディング
        PCL_ADD_NORMAL4D;        // 法線 (nx, ny, nz) + 曲率
        PCL_ADD_RGB;             // RGB情報
        float intensity;         // 強度
        float curvature;     // 曲率を明示的に追加
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    namespace io {
        template<>
        int loadPCDFile<PointXYZIRGBNormal>(
            const std::string &file_name, 
            pcl::PointCloud<PointXYZIRGBNormal> &cloud);
    }
}

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGBNormal,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
    (float, rgb, rgb)
    (float, intensity, intensity)
)

#endif // CUSTOM_POINT_TYPES_H