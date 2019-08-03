#include<fstream>
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "fisheyeimg.h"
using namespace std;


Fisheye::Fisheye() {
	img_w = 1280;
    img_h = 720;
    setIntrinsics();
}

void Fisheye::setIntrinsics() {
	fx4 = intrinsics4[0][0];
	fy4 = intrinsics4[1][1];
	cx4 = intrinsics4[0][2];
	cy4 = intrinsics4[1][2];
}

float Fisheye::getdistortcoeff0(int id){
	switch(id){
		case 0:
			return distortion_coeffs1;
		case 1:
			return distortion_coeffs2;
		case 2:
			return distortion_coeffs3;
		case 3:
			return distortion_coeffs4;
		default:
			cout<<"  ERROR allow 0~3 , request "<<id<<endl;
			exit(1);
	}
}

void Fisheye::getK0(int id,float (&intr)[3][3]){
	switch(id){
		case 0:
			for(int i=0;i<9;i++){*(*(intr+i/3)+i%3)=*(*(intrinsics1+i/3)+i%3);}
			break;
		case 1:
			for(int i=0;i<9;i++){*(*(intr+i/3)+i%3)=*(*(intrinsics2+i/3)+i%3);}
			break;
		case 2:
			for(int i=0;i<9;i++){*(*(intr+i/3)+i%3)=*(*(intrinsics3+i/3)+i%3);}
			break;
		case 3:
			for(int i=0;i<9;i++){*(*(intr+i/3)+i%3)=*(*(intrinsics4+i/3)+i%3);}
			break;
		default:
			cout<<"  ERROR allow 0~3 , request "<<id<<endl;
			exit(1);
	}
} 

void Fisheye::getIntrinsics(float& fx, float& fy, float& cx, float& cy) {
	fx = fx4; fy = fy4; cx = cx4; cy = cy4;
}

void Fisheye::getImagesize(int& wid, int& height) {
	wid = img_w;
	height = img_h;
}


bool Fisheye::distortion(float u_p[2], float d_p[2], float fx, float fy, float cx, float cy, float w){
		
	u_p[0] = (u_p[0] - cx) / fx;
	u_p[1] = (u_p[1] - cy) / fy;
		
	double mul2tanwby2 = tan(w / 2.0) * 2.0;

	// Calculate distance from point to center.
	double r_u = sqrt(u_p[0]*u_p[0] + u_p[1]*u_p[1]);
	if (mul2tanwby2 == 0 || r_u == 0) {
		return false;
	}
	// Calculate undistorted radius of point.
	double kMaxValidAngle = 89.0;
	
	double r_d;
	r_d = atan(r_u * mul2tanwby2) / (w*r_u) ;
	if (fabs(r_d * w) <= kMaxValidAngle) {
		//1
	} else {
		return false;
	}

	d_p[0] = u_p[0] * r_d;
	d_p[1] = u_p[1] * r_d;
	
	d_p[0] = d_p[0]* fx + cx;
	d_p[1] = d_p[1]* fy + cy;
	
	return true;
}

cv::Mat Fisheye::undistortimg(const cv::Mat&img,const int camidx0){
		
	cv::Mat uimg = cv::Mat::zeros(cv::Size(img.cols,img.rows), CV_8UC3);
	float intrinsics[3][3];
    getK0(camidx0, intrinsics);
	float distortion_coeffs=getdistortcoeff0(camidx0);
	for (int i=0; i< uimg.rows; ++i) {
		for(int j=0; j< uimg.cols; ++j) {
			float u_p[] = {float(j),float(i)};//x,y
			float d_p[2];
			distortion(u_p, d_p,
			intrinsics[0][0],intrinsics[1][1],intrinsics[0][2],intrinsics[1][2],distortion_coeffs);

			int jj =floor( d_p[0]);
			int ii = floor(d_p[1]); 
			if (ii>=0 && ii <= img.rows-2 && jj>=0 && jj <= img.cols-2) {
				int jj1=jj+1;
				int ii1=ii+1;
				float ki=d_p[1]-ii;
				float kj=d_p[0]-jj;
				cv::Vec3b v00 = img.at<cv::Vec3b>(ii,jj);
				cv::Vec3b v10 = img.at<cv::Vec3b>(ii1,jj);
				cv::Vec3b v01 = img.at<cv::Vec3b>(ii,jj1);
				cv::Vec3b v11 = img.at<cv::Vec3b>(ii1,jj1);
				cv::Vec3b v;
				for(int i3=0;i3<3;i3++){
				    v[i3]=uchar(int((1-ki)*(1-kj)*v00[i3]+(ki)*(1-kj)*v10[i3]+(1-ki)*(kj)*v01[i3]+(ki)*(kj)*v11[i3]));
				}
				uimg.at<cv::Vec3b>(i,j) = v;
			}
		}
	}
	return uimg;
}