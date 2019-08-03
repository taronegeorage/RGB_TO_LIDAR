#include "utility.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>


#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/image_encodings.h>
// #include <image_transport/image_transport.h>s
// #include <compressed_image_transport/compression_common.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include<pcl/visualization/cloud_viewer.h>
// #include <fstream>

typedef pcl::PointXYZI  PointType;
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

class twjPcNode{

private:
    ros::NodeHandle nh;
    ros::Subscriber subcld1;

pcl::PointCloud<PointType>::Ptr cloudvelo;
pcl::PointCloud<PointType>::Ptr cloudsegmpure;
pcl::PointCloud<PointType>::Ptr cloudlesssharp;
pcl::PointCloud<PointType>::Ptr cloudlesssharplast;
pcl::PointCloud<PointType>::Ptr cloudsurround;

    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

ofstream ofs4;
string st;
public:
  twjPcNode(): nh("~"){
  nh.getParam("/savepath",st);
  ofs4.open(st+"timepc.txt");

  subcld1=  nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, &twjPcNode::cldHandler1, this);

  cloudvelo.reset(new pcl::PointCloud<PointType>());

  }

  void cldHandler1(const sensor_msgs::PointCloud2ConstPtr& cld_msg) {
	  static int ii=-1;
		ii++;
    cloudvelo->clear();
    pcl::fromROSMsg(*cld_msg, *cloudvelo);
    stringstream ss;
    ss<<st<<"velodyne/"<<setw(6) << setfill('0')<<ii<<".pcd";
    pcl::io::savePCDFileASCII (ss.str(), *cloudvelo);
    ss<<"   saved "<<cloudvelo->points.size ()<<"points";
    ROS_INFO(ss.str().c_str());
    ofs4<<cld_msg->header.stamp.sec<<"."<<setw(9) << setfill('0')<<cld_msg->header.stamp.nsec<<endl;
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag2other");
    
    twjPcNode tim;

    ROS_INFO("\033[1;32m---->\033[0m twjPcNode Started.");

    
    ros::spin();

    return 0;
}
