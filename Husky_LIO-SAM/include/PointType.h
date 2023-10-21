#ifndef __LIO_SAM_POINTTYPE_H__
#define __LIO_SAM_POINTTYPE_H__

namespace pcl {
    struct PointNormalRTL {
        PCL_ADD_POINT4D;               // xyz + intensity
        PCL_ADD_INTENSITY;
        PCL_ADD_NORMAL4D; // normal + curvature
        // PCL_ADD_CURVATURE;
        uint16_t ring;
        float time;
        uint32_t label;                        // label
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointNormalRTL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, normal_x, normal_x)
                                  (float, normal_y, normal_y)
                                  (float, normal_z, normal_z)
                                //   (float, curvature, curvature)
                                  (uint16_t, ring, ring)
                                  (float, time ,time)
                                  (uint32_t, label, label))


// typedef pcl::PointXYZI PointType;
typedef pcl::PointNormalRTL PointType;

#endif