//
// Created by lzw on 19-2-13.
//

/*
 * Extract pole-based landmarks from pcd file built from rgbdl_builder.
 * Based on pcl tutorial.
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


int
main (int argc, char** argv)
{
    if(argc < 3){
        std::cout << "Usage: pcl_generate [path_to_pcd] [param_distance(m)]" << std::endl;
        return 0;
    }
    std::cout << "Distance: " << (argv[2]) << "m" << std::endl;

    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
    reader.read (argv[1], *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

//    // Create the filtering object: downsample the dataset using a leaf size of 1cm
//    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
//    vg.setInputCloud (cloud);
//    vg.setLeafSize (0.01f, 0.01f, 0.01f);
//    vg.filter (*cloud_filtered);
//    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered = cloud;        // stop downsampling. as it will erase a channel( which stores label value )

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance (atof(argv[2]));         // Condition1. we demand distance > 20cm
    ec.setMinClusterSize (1000);                    // Condition2. we need more than 1000 points
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    // final point cloud should be 2D! We store them in a new data structure.
    // Use point cloud to store temply. Set Y axis to be zero.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr compact_map (new pcl::PointCloud<pcl::PointXYZRGBA>);         // using RGB to represent different labels

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        // output those cluster ones.
        double aver_x = 0, aver_z = 0;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
        int jump_size = 0;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            // we jump those pole points that are higher than a threshold.
            double y = cloud_filtered->points[*pit].y;
            if(cloud_filtered->points[*pit].a == 9 && (y>-113 || y<-118)) {
                jump_size++;
                continue;

            }
            aver_x += cloud_filtered->points[*pit].x;
            aver_z += cloud_filtered->points[*pit].z;
        }

        if(abs(aver_x)<0.1 && abs(aver_z)<0.1) continue;

        aver_x /= double(it->indices.size()-jump_size);
        aver_z /= double(it->indices.size()-jump_size);
        pcl::PointXYZRGBA landmark;
        landmark.x = aver_x;
        landmark.y = -112.650;             // Attension: we need to set it as the same as the hight of car.     0002: -114
        landmark.z = aver_z;
        landmark.a = cloud_filtered->points[it->indices[0]].a;
        // set color according to label.
        assert(int(cloud_filtered->points[it->indices[0]].a) != 0);
        switch ( int(cloud_filtered->points[it->indices[0]].a) )
        {
            case 7 : landmark.r = 255;break;
            case 8 : landmark.g = 255;break;
            case 9 : landmark.b = 255;break;
            default: break;
        }

        compact_map->points.push_back(landmark);

        std::cout << "[" << j << "] PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        j++;
    }

    compact_map->width = compact_map->points.size ();
    compact_map->height = 1;
    compact_map->is_dense = true;
    std::stringstream ss;
    ss << "compact_map.pcd";
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