#include "utility.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "fisheyeimg.h"
//#include <basic_image_processor.hpp>
// #include<pcl/visualization/cloud_viewer.h>
// #include<iostream>
// #include<fstream>
class twjImgNode{

public:

  twjImgNode(): nh("~"){
    fe = new Fisheye();
    twjk=0;
    nh.getParam("/savepath",st);
    ofs1.open(st+"time1.txt");
    ofs2.open(st+"time2.txt");
    ofs3.open(st+"time3.txt");
    //ofs4.open(st+"time4.txt");
	  //image_transport::ImageTransport it(nh);
    // subimg = it.subscribe("/camera/post1", 5, &twjImgNode::imgHandler,this);
	  //subcld=  nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1, &twjImgNode::cldHandler, this);//以后加ground
    //pubnewimg=nh.advertise<sensor_msgs::Image>("/twjpubnewimg",5);
    //pubimg=nh.advertise<sensor_msgs::Image>("/twjpubimg",5);
    //subimgros=  nh.subscribe<sensor_msgs::Image>("/twjpubros", 1, &twjImgNode::imgrosHandler, this);
  
    // subimg1=  nh.subscribe<sensor_msgs::CompressedImage>("/camera/post1/compressed", 20, &twjImgNode::imgHandler1, this);
    // subimg2=  nh.subscribe<sensor_msgs::CompressedImage>("/camera/post2/compressed", 20, &twjImgNode::imgHandler2, this);
    // subimg3=  nh.subscribe<sensor_msgs::CompressedImage>("/camera/post3/compressed", 20, &twjImgNode::imgHandler3, this);
    subimg4=  nh.subscribe<sensor_msgs::CompressedImage>("/camera/post4/compressed", 1000, &twjImgNode::imgHandler4, this);
  }
  Fisheye* fe;
  void imgHandler1(const sensor_msgs::CompressedImageConstPtr& img_msg) {
    static int ii=-1;
		ii++;
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg,"bgr8");
    stringstream ss;
    ss<<st<<"1/"<<setw(6) << setfill('0')<<ii<<".jpg";

 	  ROS_INFO(ss.str().c_str());
     //cv::imshow("AA",ptr->image);
    cv::imwrite(ss.str(),ptr->image);
    ofs1<<img_msg->header.stamp.sec<<"."<<setw(9) << setfill('0')<<img_msg->header.stamp.nsec<<endl;
  }
  void imgHandler2(const sensor_msgs::CompressedImageConstPtr& img_msg) {
    static int ii=-1;
		ii++;
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg,"bgr8");
    stringstream ss;
    ss<<st<<"2/"<<setw(6) << setfill('0')<<ii<<".jpg";
 	  ROS_INFO(ss.str().c_str());
    //cv::imshow("AA",ptr->image);
    cv::imwrite(ss.str(),ptr->image);
    ofs2<<img_msg->header.stamp.sec<<"."<<setw(9) << setfill('0')<<img_msg->header.stamp.nsec<<endl;
  }
  void imgHandler3(const sensor_msgs::CompressedImageConstPtr& img_msg) {
    static int ii=-1;
		ii++;
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg,"bgr8");
    stringstream ss;
    ss<<st<<"3/"<<setw(6) << setfill('0')<<ii<<".jpg";
 	  ROS_INFO(ss.str().c_str());
    cv::imwrite(ss.str(),ptr->image);
    ofs3<<img_msg->header.stamp.sec<<"."<<setw(9) << setfill('0')<<img_msg->header.stamp.nsec<<endl;
  }
  void imgHandler4(const sensor_msgs::CompressedImageConstPtr& img_msg) {
    static int ii=-1;
		ii++;
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg,"bgr8");
    stringstream ss;
    ss<<st<<"4_distort/"<<setw(6) << setfill('0')<<ii<<".jpg";
 	  ROS_INFO(ss.str().c_str());
    cv::imwrite(ss.str(),fe->undistortimg(ptr->image, 3));
    //ofs4<<img_msg->header.stamp.sec<<"."<<setw(9) << setfill('0')<<img_msg->header.stamp.nsec<<endl;
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber subcld;
  ros::Publisher pubnewimg;
  ros::Publisher pubimg;
  // ros::Subscriber subimg1;
  // ros::Subscriber subimg2;
  // ros::Subscriber subimg3;
  ros::Subscriber subimg4;
  ofstream ofs1;
  ofstream ofs2;
  ofstream ofs3;
  ofstream ofs4;
  int twjk;
  string st;
};
/*
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("imgc");
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag2img");
    
    twjImgNode tim;
//       cv::Mat image = cv::imread("/home/twj/collect/legoviewws/src/legoview/LeGO-LOAM/2019.png", CV_LOAD_IMAGE_COLOR);
//       cv::imshow("b",image);
//       cv::waitKey(1000);
    ROS_INFO("\033[1;32m---->\033[0m twjImgNode Started.");
    ros::spin();
    return 0;
}
