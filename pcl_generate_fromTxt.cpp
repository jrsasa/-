//
// Created by lzw on 19-3-30.
//

/*
 * Generate point cloud pcd file from txt. The text comes from shift+left_click in pcl_viewer.
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
#include <fstream>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

using namespace std;

std::vector< pcl::PointXYZRGBA > readFromText(char* path)
{
    vector< pcl::PointXYZRGBA > points;

    ifstream fin(path);
    string line;

    while( getline(fin, line) )
    {
        vector<double> data;
        vector<string> s;
        boost::split( s, line, boost::is_any_of( ",[]" ), boost::token_compress_on );

        pcl::PointXYZRGBA p;

        p.x = std::stod(s[1]);
        p.y = std::stod(s[2]);
        p.z = std::stod(s[3]);

        points.push_back(p);
    }

    return points;
}

int
main (int argc, char** argv)
{
    if(argc < 2){
        std::cout << "Usage: " << argv[0] << " [path_to_txt]" << std::endl;
        return 0;
    }

    std::vector< pcl::PointXYZRGBA > points;
    // read from text, store as Point
    points = readFromText( argv[1] );

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr compact_map (new pcl::PointCloud<pcl::PointXYZRGBA>);         // using RGB to represent different labels
    for( auto p: points){

        // change the coordinites?
        pcl::PointXYZRGBA landmark;
        landmark.x = p.x;
        landmark.y = p.y;
        landmark.z = 18.0;

        landmark.a = 9;  // as pole

        switch ( landmark.a )
        {
            case 7 : landmark.r = 255;break;
            case 8 : landmark.g = 255;break;
            case 9 : landmark.b = 255;break;
            default: break;
        }

        compact_map->points.push_back(landmark);
    }

    compact_map->width = compact_map->points.size ();
    compact_map->height = 1;
    compact_map->is_dense = true;

    std::stringstream ss;
    ss << "compact_map_poles.pcd";

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGBA> (ss.str (), *compact_map, false); //*
    std::cout << "Write compact_map.pcd, total landmarks: " << compact_map->width << std::endl;
    
    return (0);
}


/*
 * Old version for outputting segmentation result
 */
//int j = 0;
//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//{
//// output those cluster ones.
//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
//for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
//cloud_cluster->width = cloud_cluster->points.size ();
//cloud_cluster->height = 1;
//cloud_cluster->is_dense = true;
//
//std::cout << "[" << j << "] PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//std::stringstream ss;
//ss << "cloud_cluster_" << j << ".pcd";
//writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_cluster, false); //*
//j++;
//}