/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-09-04 13:10:11
 */

#include "ground_segment.h"


// GroundSegmentation::GroundSegmentation(PointCloudType::Ptr incloud,GroundSegmentationMsg msg)
// {
//     _threshold=msg.segthres();
//   _cloudptrlist.resize(6);

//   #pragma omp parallel for schedule(runtime)
//   for (int i = 0; i < _cloudptrlist.size(); ++i)
//   {
//       _cloudptrlist[i].reset(new PointCloudType);
//   }

//   //分段进行平面分割
//   for (int i = 0; i < incloud->points.size(); ++i)
//   {
//       if(incloud->points[i].x<=15&&incloud->points[i].x>=0&&abs(incloud->points[i].y)<30)
//       {
//           _cloudptrlist[0]->points.push_back(incloud->points[i]);
//       }
//       if(incloud->points[i].x>=-15&&incloud->points[i].x<0&&abs(incloud->points[i].y)<30)
//       {
//           _cloudptrlist[1]->points.push_back(incloud->points[i]);
//       }
//       if(incloud->points[i].x>15&&incloud->points[i].x<=30&&abs(incloud->points[i].y)<30)
//       {
//           _cloudptrlist[2]->points.push_back(incloud->points[i]);
//       }
//       if(incloud->points[i].x>=-30&&incloud->points[i].x<-15&&abs(incloud->points[i].y)<30)
//       {
//           _cloudptrlist[3]->points.push_back(incloud->points[i]);
//       }
//       if(incloud->points[i].x<=40&&incloud->points[i].x>=30&&abs(incloud->points[i].y)<30)
//       {
//           _cloudptrlist[4]->points.push_back(incloud->points[i]);
//       }
//       if(incloud->points[i].x<=-30&&incloud->points[i].x>=-40&&abs(incloud->points[i].y)<30)
//       {
//           _cloudptrlist[5]->points.push_back(incloud->points[i]);
//       }
//   }
// }

GroundSegmentation::GroundSegmentation(PointCloudType::Ptr incloud)
{
  _threshold = 0.35;
  _cloudptrlist.resize(6);

  #pragma omp parallel for schedule(runtime)
  for (int i = 0; i < _cloudptrlist.size(); ++i)
  {
      _cloudptrlist[i].reset(new PointCloudType);
  }

  //分段进行平面分割
  for (int i = 0; i < incloud->points.size(); ++i)
  {
      if(incloud->points[i].x<=15&&incloud->points[i].x>=0&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[0]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x>=-15&&incloud->points[i].x<0&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[1]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x>15&&incloud->points[i].x<=30&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[2]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x>=-30&&incloud->points[i].x<-15&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[3]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x<=40&&incloud->points[i].x>=30&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[4]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x<=-30&&incloud->points[i].x>=-40&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[5]->points.push_back(incloud->points[i]);
      }
  }
}


void GroundSegmentation::extractGround(PointCloudType::Ptr outCloud,
                                   PointCloudType::Ptr inputCloud,
                                   pcl::PointIndices::Ptr Indices,
                                   bool setNeg)
{
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(Indices);
    extract.setNegative(setNeg);
    extract.filter(*outCloud);
}

void GroundSegmentation::planeSeg(PointCloudType::Ptr cloud,
                              pcl::ModelCoefficients::Ptr coefficients,
                              pcl::PointIndices::Ptr planeIndices)
{
    pcl::SACSegmentation<pcl::PointXYZI> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(_threshold);
    segmentation.setMaxIterations(800);
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);
}


void GroundSegmentation::groundfilter(PointCloudType::Ptr groundpoints,PointCloudType::Ptr non_groundpoints)
{
    groundpoints->points.clear();
    non_groundpoints->points.clear();
   
    for (int i = 0; i < _cloudptrlist.size()-2; i++)
    {
        std::cout << i << std::endl;
        
        PointCloudType::Ptr ground_i(new  PointCloudType);
        PointCloudType::Ptr ground_no_i(new  PointCloudType);
        pcl::ModelCoefficients::Ptr modelCoe1(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr indices1(new pcl::PointIndices);
        planeSeg(_cloudptrlist[i],modelCoe1,indices1);
        
        extractGround(ground_i,_cloudptrlist[i],indices1,false);
        std::cout << ground_i->points.size() << std::endl;

        extractGround(ground_no_i,_cloudptrlist[i],indices1,true);
        std::cout << ground_no_i->points.size() << std::endl;

        *groundpoints+=*ground_i;

        *non_groundpoints+=*ground_no_i;
    }
    
}
