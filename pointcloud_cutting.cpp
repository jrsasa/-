//
// Created by liaoziwei on 19-3-25.
//

/*
 * Get part of the pointclouds, giving the rectangle coordinates.
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <string>
#include <iostream>

int main (int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage:() [path_to_pcd] [output] " << std::endl;//[x0] [y0] [x1] [y1] [x2] [y2] [x3] [y3]切割矩形时所用
        return 0;
    }
   // std::cout << "Loading pcd: " << (argv[1]) << std::endl;

    //double x0 = std::atof((argv[3]));
    //double y0 = std::atof((argv[4]));
    //double x1 = std::atof((argv[5]));
    //double y1 = std::atof((argv[6]));
    //double x2 = std::atof((argv[7]));
    //double y2 = std::atof((argv[8]));
    //double x3 = std::atof((argv[9]));
    //double y3 = std::atof((argv[10]));

    //std::printf("Range: P0[ %f, %f ], P1[ %f, %f] ,P2[ %f, %f],P3[ %f, %f]\n", x0,y0,x1,y1,x2,y2,x3,y3);

    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_output(
            new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(argv[1], *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

    // traversal them.
    int number = 0;
    for(auto p : cloud->points )
    {
       bool bInRegion = false;
         /*int a=(x1-x0)*(p.y-y0)-(y1-y0)*(p.x-x0);
        int b=(x2-x1)*(p.y-y1)-(y2-y1)*(p.x-x1);
        int c=(x3-x2)*(p.y-y2)-(y3-y2)*(p.x-x2);
        int d=(x0-x3)*(p.y-y3)-(y0-y3)*(p.x-x3);
        if((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0))*/
         if((p.z>10)&&(p.z<30))
                bInRegion = true;

        if(bInRegion)
        {
            cloud_output->points.push_back( p );
        }

        number ++;
        if( number % 1000 == 0)
        {
            std::cout << "Total Number: " << number << ", all: " << cloud->points.size() << std::endl;
        }
    }


    // save pcd
    pcl::PCDWriter writer;

    cloud_output->width = cloud_output->points.size ();
    std::cout << "cloud_output->width: " <<cloud_output->width<<std::endl;
    cloud_output->height = 1;
    cloud_output->is_dense = true;
    writer.write<pcl::PointXYZ> (argv[2], *cloud_output, false); //*
    std::cout << "Save to: " << argv[2] << " , total landmarks: " << cloud_output->width << std::endl;



}
