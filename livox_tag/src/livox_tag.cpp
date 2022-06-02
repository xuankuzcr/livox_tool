#include <iostream>
#include <set>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <time.h>
#include "livox_ros_driver/CustomMsg.h"
#include "common.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "highgui.h"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;

struct pointData
{
    float x;
    float y;
    float z;
    int tag;
   //int i;
};

float dilation_size = 0;
vector<pointData> vector_data;
set<int> tagList; // 构造空的l1

livox_ros_driver::CustomMsg livox_cloud;
string input_bag_path, output_path,output_path1,output_path2;
int threshold_lidar, data_num;

void loadAndSavePointcloud(int index);

//*************************bag Points to pcd Points**************************************//
void loadAndSavePointcloud(int index) 
{
    string path = input_bag_path + std::to_string(index) + ".bag";
    fstream file_;
    file_.open(path, ios::in);
    if (!file_) 
    {
        cout << "File " << path << " does not exit" << endl;
        return;
    }
    ROS_INFO("Start to load the rosbag %s", path.c_str());
    rosbag::Bag bag;
    try 
    {
        bag.open(path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) 
    {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }
    
    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg")); 
    // types.push_back(string("sensor_msgs/PointCloud2")); 
    rosbag::View view(bag, rosbag::TypeQuery(types));
    int cloudCount = 0;

    for (const rosbag::MessageInstance& m : view) 
    {
        livox_cloud = *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
  
        //for(uint i = 0; i < livox_cloud.point_num; ++i)
        for(uint i = 0; i < livox_cloud.point_num; ++i)
        {
            pointData myPoint;
            myPoint.x = livox_cloud.points[i].x;
            myPoint.y = livox_cloud.points[i].y;
            myPoint.z = livox_cloud.points[i].z;
            myPoint.tag = livox_cloud.points[i].tag;
            if(livox_cloud.points[i].tag!=16)
                tagList.insert(livox_cloud.points[i].tag);
           // myPoint.i = livox_cloud.points[i].reflectivity;
          //  if(myPoint.z >= -2.2 && myPoint.z <= 0.5)
          //   {
           //     if(myPoint.y>= 4.5 && myPoint.y <= 7)
           //     {
            // vector_data.push_back(myPoint);
           //     }
          //  }
            // if(livox_cloud.points[i].tag!=16 && livox_cloud.points[i].tag!=32 && livox_cloud.points[i].tag!=48)
            if(livox_cloud.points[i].tag==32 || livox_cloud.points[i].tag==48)
                vector_data.push_back(myPoint);
            //vector_data.push_back(myPoint);
        }
        ++cloudCount;
        if (cloudCount >= threshold_lidar) 
        {
           unsigned long timebase_ns = livox_cloud.timebase;
           if(vector_data.size()==0) 
            cout<< "This scan does not have tag 48..."<<endl;
           else
            {
                for(auto &v:vector_data)
                {
                    cout<<"LiDAR Point: "<< v.x << ", "<< v.y << ", " << v.z << ", " << v.tag <<endl;
                }
            }

           vector_data.clear();
           cloudCount = 0;
           // break;
        }
    }
    for (auto& e : tagList)
		cout << e << " ";
}

//***********************getparameters from launch***********************//
void getParameters() 
{
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path)) 
    {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    }
    else 
    {
        cout << input_bag_path << endl;
    }
    if (!ros::param::get("output_pcd_path", output_path)) 
    {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("output_range_path", output_path1)) 
    {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("output_range_path2", output_path2))
    {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("threshold_lidar", threshold_lidar)) 
    {
        cout << "Can not get the value of threshold_lidar" << endl;  
        exit(1);         
    }
    if (!ros::param::get("data_num", data_num))    
    {
        cout << "Can not get the value of data_num" << endl;
        exit(1);
    }
}

//****************************Main***********************************//
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "bag2range");
    getParameters();

    for (int i = 0; i < data_num; ++i) 
    {
        loadAndSavePointcloud(i);
    }
    ROS_INFO("Finish all!");
    return 0;
}


