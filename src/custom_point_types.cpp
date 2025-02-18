#include "custom_point_types.h" // あなたのカスタムポイント定義
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/point_representation.h>

#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/registration/impl/correspondence_estimation.hpp>

namespace pcl {
    namespace io {
        template<>
        int loadPCDFile<PointXYZIRGBNormal>(const std::string &file_name, pcl::PointCloud<PointXYZIRGBNormal> &cloud)
        {
            // std::cout << "load PCD File... PointXYZIRGBNormal" << std::endl;
            pcl::PCLPointCloud2 blob;
            int res = pcl::io::loadPCDFile(file_name, blob);
            if (res != 0) return res;

            // PCLPointCloud2からカスタム型への変換
            cloud.clear();
            cloud.resize(blob.width * blob.height);

            // フィールドのインデックスを探す
            int x_idx = -1, y_idx = -1, z_idx = -1, rgb_idx = -1, intensity_idx = -1;
            for (size_t d = 0; d < blob.fields.size(); ++d) {
                if (blob.fields[d].name == "x") x_idx = d;
                else if (blob.fields[d].name == "y") y_idx = d;
                else if (blob.fields[d].name == "z") z_idx = d;
                else if (blob.fields[d].name == "rgb") rgb_idx = d;
                else if (blob.fields[d].name == "intensity") intensity_idx = d;
            }
            // std::cout << x_idx << " " << y_idx << " " << z_idx << " " << rgb_idx << " " << intensity_idx << std::endl;
            uint8_t* point_data = blob.data.data();
            for (size_t i = 0; i < cloud.size(); ++i) {
                // XYZ
                memcpy(&cloud.points[i].x, point_data + blob.point_step * i + blob.fields[x_idx].offset, sizeof(float));
                memcpy(&cloud.points[i].y, point_data + blob.point_step * i + blob.fields[y_idx].offset, sizeof(float));
                memcpy(&cloud.points[i].z, point_data + blob.point_step * i + blob.fields[z_idx].offset, sizeof(float));

                // RGB
                if (rgb_idx >= 0) {
                    float rgb_float;
                    memcpy(&rgb_float, point_data + blob.point_step * i + blob.fields[rgb_idx].offset, sizeof(float));
                    // // 生のfloat値とバイトパターンを表示
                    // std::cout << "RGB float value: " << rgb_float << std::endl;
                    // uint8_t* bytes = (uint8_t*)&rgb_float;
                    // std::cout << "Bytes: " 
                    //         << std::hex 
                    //         << (int)bytes[0] << " " 
                    //         << (int)bytes[1] << " " 
                    //         << (int)bytes[2] << " " 
                    //         << (int)bytes[3] << std::endl;

                    uint8_t* rgb_bytes = reinterpret_cast<uint8_t*>(&rgb_float);
                    // バイト順序を修正
                    memcpy(&cloud.points[i].r, &rgb_bytes[0], sizeof(uint8_t));
                    memcpy(&cloud.points[i].g, &rgb_bytes[1], sizeof(uint8_t));
                    memcpy(&cloud.points[i].b, &rgb_bytes[2], sizeof(uint8_t));
                }
                if (intensity_idx >= 0) {
                    memcpy(&cloud.points[i].intensity, point_data + blob.point_step * i + blob.fields[intensity_idx].offset, sizeof(float));
                }
                // std::cout << cloud.points[i].x <<  " " <<  cloud.points[i].y << " " << cloud.points[i].z  << " "  << (int)cloud.points[i].r << " " << (int)cloud.points[i].g << " " << (int)cloud.points[i].b  << " " << cloud.points[i].intensity << std::endl;
            }
            return 0;
        }
    }
    template class PCLBase<PointXYZIRGBNormal>;
    template class Filter<PointXYZIRGBNormal>;
    template class VoxelGrid<PointXYZIRGBNormal>;

    template class KdTreeFLANN<PointXYZIRGBNormal>;
    namespace search {
        template class KdTree<PointXYZIRGBNormal>;
    }

    template class Registration<PointXYZIRGBNormal, PointXYZIRGBNormal>;
    namespace registration {
        template class CorrespondenceEstimationBase<PointXYZIRGBNormal, PointXYZIRGBNormal, float>;
    }

    template void removeNaNFromPointCloud<PointXYZIRGBNormal>(
        const pcl::PointCloud<PointXYZIRGBNormal>&,
        pcl::PointCloud<PointXYZIRGBNormal>&,
        std::vector<int>&);

}

