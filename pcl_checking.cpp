//
//created by zzd on 19-4-20
/*plot calibrated points in the point cloud pcl file 
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
    if(argc < 3){
        std::cout << "Usage: " << argv[0] << " [path_to_txt]  [path_to_pcd]  " <<std::endl;//3 
        return 0;
    }

    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //original cloud
    reader.read(argv[2], *cloud);
  



    std::vector< pcl::PointXYZRGBA > points;
    // read from text, store as Point
    points = readFromText( argv[1] );                  
    std::cout << "width of cloud " << cloud->width<< "height of cloud " << cloud->height<< "total number of points "<< cloud->points.size() << std::endl;                            

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr compact_map (new pcl::PointCloud<pcl::PointXYZRGBA>);         // using RGB to discriminate original points from landmarks 

    int number = 0;
    for(auto p : cloud->points )
    {
        bool bInRegion = false;
            if((p.z>20)&&(p.z<30))
                bInRegion = true;

        if(bInRegion)
        {
            pcl::PointXYZRGBA original_points;
            original_points.x = p.x;
           original_points.y = p.y;
            original_points.z = p.z;
            original_points.r = 255;  // as pole

        
           compact_map->points.push_back(original_points);
        }

        number ++;
        if( number % 1000 == 0)
        {
            std::cout << "original Number: " << number << ", all: " << cloud->points.size() << std::endl;
        }
    }

int number_l=0;
    for( auto p: points){

        // change the coordinites?
        pcl::PointXYZRGBA landmark;
        landmark.x = p.x;
        landmark.y = p.y;
        landmark.z = 18.0;

        landmark.b = 255;  
       
        compact_map->points.push_back(landmark);
        number_l ++;
 std::cout << " Number of landmarks: " << number_l << std::endl;
             
        
        
      
    }

    compact_map->width = compact_map->points.size() ;
    compact_map->height = 1;
    compact_map->is_dense = true;
   
     std::cout << "total number " << number+number_l<< ", all: " << cloud->points.size() << std::endl;

    std::stringstream ss;
    ss << "calibrated_compact_map_poles.pcd";

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGBA> (ss.str (), *compact_map, false); 
    std::cout << "Write calibrated_compact_map.pcd, total landmarks: " << compact_map->width << std::endl;
    
    return (0);
}

