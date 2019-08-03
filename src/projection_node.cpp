// laser /home/cj/Downloads/outdoor_data/pc/velodyne/000441.pcd
//img /home/cj/Downloads/outdoor_data/image/4_distort/000449.jpg

#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "fisheyeimg.h"

using namespace std;

int main(int argc, char **argv)
{
    Fisheye *f = new Fisheye();
    float fx, fy, cx, cy;
    f->getIntrinsics(fx, fy, cx, cy);
    int w, h;
    f->getImagesize(w, h);
    Eigen::Matrix4f Tcl = Eigen::Matrix4f::Identity();
    f->getTcl(Tcl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cld(new pcl::PointCloud<pcl::PointXYZ>);
	if(pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/cj/Downloads/outdoor_data/pc/velodyne/000441.pcd", *cld) == -1) {
		PCL_ERROR("CANNOT READ FILE");
		return -1;
	}
	cout << "Loaded " << cld->width << "*" << cld->height << " data points." << endl;
    pcl::transformPointCloud(*cld, *cld, Tcl);
    std::cout << Tcl.matrix() << std::endl;
    pcl::io::savePCDFileBinary("000441_new.pcd", *cld );
    cv::Mat img = cv::imread("/home/cj/Downloads/outdoor_data/image/4_distort/000449.jpg");

    // combine
    typedef pcl::PointXYZRGB PointT;
    pcl::PointCloud<PointT> newpc;
    newpc.resize(cld->width*cld->height);
    int count = 0;
    float maxX = 25.0, maxY = 6.0, minZ = -1.4;
    for(auto& point : cld->points) {
        if(point.z <= 0)
            continue;
        // if(point.x > maxX || point.x < 0.0 || abs(point.y) > maxY || point.z < minZ)
        //     continue;
        int u = (int)(point.x*fx/point.z+cx);
        int v = (int)(point.y*fy/point.z+cy);
        //std::cout << u << " " << v << std::endl;
        if(u < 0 || v < 0 || u > w || v > h)
            continue;
        newpc.points[count].x = point.x;
        newpc.points[count].y = point.y;
        newpc.points[count].z = point.z;
        newpc.points[count].b = img.data[v*img.step+u*img.channels()];
        newpc.points[count].g = img.data[v*img.step+u*img.channels()+1];
        newpc.points[count].r = img.data[v*img.step+u*img.channels()+2];
        count++;
    }
    newpc.height = 1;
    newpc.width = count;
    newpc.resize(count);
    newpc.is_dense = false;

    pcl::io::savePCDFileBinary("newpc.pcd", newpc );

    return 0;
}
