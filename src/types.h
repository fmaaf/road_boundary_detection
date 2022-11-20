#ifndef TYPES_H_
#define TYPES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>



typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef std::vector<PointCloudType::Ptr > CloudPtrList;
typedef std::vector<PointCloudType> CloudList;


typedef std::pair<size_t, size_t> IndexRange;
typedef std::vector<IndexRange> scanIndices;

#endif
