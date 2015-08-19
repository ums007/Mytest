#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <cmath>
#include <ctime>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>

#define PI 3.14159265
double car_len=10,nvel=20,counter=91;
double vel=0;
clock_t start, end;
double msecs=0;
double xi = 600;
double yi = 500;
double xf = xi;
double yf = yi;
double h=xi;
double k=yi;


double p=600;
double q=150;
double r=100;
int vel_flag = 0;
int start_flag = 0;
int search_flag = 0; 
int create_flag = 0;
double final_coor[50];
int ic=-2; 

static const std::string OPENCV_WINDOW_LEFT = "Image window left";
static const std::string window_name = "Image";
using namespace std;
using namespace cv;
int lowThreshold=0;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == EVENT_LBUTTONDOWN ){
	ic = ic+2;
final_coor[ic] = y;
final_coor[ic+1] = x;
vel_flag=0;
start_flag=1;

ofstream way;
way.open("waypoint.txt", ios::app);

way <<y<<'\t' ;
way <<x<<'\n' ;
way.close();
search_flag=1;

}

     
}

double theta;
double alpha;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_l;
  image_transport::Publisher image_pub_l;
  class way{

float sx = final_coor[ic];
float sy = final_coor[ic+1];

  };


public:
void search(){
while(search_flag == 0){
	
	ifstream way;
	way.open("waypoint.txt",ios::in);
	way.seekg(500, ios::beg);
	
	if(!way.eof())
		{way.close();
			ROS_INFO_STREAM("wrong loop");
			
			break;
		}
	else if(way.eof())
		{ ROS_INFO_STREAM("Differnt loop");
			exit(0);
}

	
}
ifstream way;
way.open("waypoint.txt",ios::in);
//way.seekg(0, ios::beg);
if(way.eof()){ROS_INFO_STREAM("End of file detected");
way.close();}
else if(!way.eof()){ROS_INFO_STREAM("No End of file");
way.close();}

}

double velocity(double p,double q,double r,double s){
	if(start_flag==0){goto A;}
	theta = atan2((yi-final_coor[ic+1]),(xi-final_coor[ic]));
alpha = theta;
	int dist,tot_dist;
	tot_dist = sqrt(((final_coor[ic+1]-yi)*(final_coor[ic+1]-yi))+((final_coor[ic]-xi)*(final_coor[ic]-xi)));
	if (tot_dist==0)
		{goto A;}
	if(vel_flag==0)
{
	
		vel=vel+2;
		if(vel>=nvel)
			{
				vel_flag=1;
			}
		goto A;
	
}
	
	dist = sqrt(((q-s)*(q-s))+((p-r)*(p-r)));
	

if(dist<0.9*tot_dist){

	//ROS_INFO_STREAM("Next Velocity:" <<vel);
	ROS_INFO_STREAM(" Next Heading: " <<theta*180/PI);
}
else 
{	//ROS_INFO_STREAM("Next Velocity: " <<vel);
	if(((dist*100)/tot_dist)>=counter){
	vel=vel-(0.1*nvel);
	if(vel<0){vel=0;}
	if(counter==100){
		start_flag=0;
		//xi = final_coor[ic];
		//yi = final_coor[ic+1];
		xi = h;
		yi = k;
		counter = 91;
		
		goto A;
	}

	
counter=counter+1;
}
}
A:
ROS_INFO_STREAM("Next Velocity: " <<vel);
ROS_INFO_STREAM(" Next Heading: " <<theta*180/PI);
return(vel);

}
 
 void odom(Mat imgi){
	double z=0;
    char s[50];
       
    while(z<=1)
    {
       	sprintf(s,"%f",(nvel*z));
   		cv::line(imgi,Point(p,q),Point(p-(r*cos(PI*vel/nvel)),q-(r*sin(PI*vel/nvel))),cv::Scalar(255,255,255),3);
       	cv::putText(imgi,s, Point(p-(r*cos(PI*z)),q-(r*sin(PI*z))), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,255,250), 1,1);
   		circle(imgi,Point(p-(r*cos(PI*z)),q-(r*sin(PI*z))),1,Scalar(0,0,255));
   		z=z+(1.0/4);
	}
 }

void path_gen(Mat imgi){
	circle(imgi, Point(yi,xi),4,Scalar(0,255,0));
	if(start_flag==0)
		{goto B;}

	circle(imgi, Point(final_coor[ic+1],final_coor[ic]),4,Scalar(0,0,255));
	cv::putText(imgi, "STOP", Point(final_coor[ic+1]+10,final_coor[ic]), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,0,255), 2,1);
	cv::line(imgi,Point(yi,xi),Point(final_coor[ic+1],final_coor[ic]),cv::Scalar(0,255,0),1);
	B:
	cv::putText(imgi, "START", Point(yi+10,xi), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,255,0), 2,1);
	
	
}

void file_create(){
	ofstream way;
	way.open("waypoint.txt");
	way.close(); 
}


  
  ImageConverter()
    : it_(nh_)
  {
	    // Subscrive to input video feed and publish output video feed
	    image_sub_l = it_.subscribe("/usb_cam/image_raw", 1, 
	      &ImageConverter::imageCb_l, this);
	    image_pub_l = it_.advertise("/usb_cam/image_convert", 1);
	    

	    cv::namedWindow(OPENCV_WINDOW_LEFT);
	   // remove("waypoint.txt");
   
  }

  ~ImageConverter()
  {

   	cv::destroyWindow(OPENCV_WINDOW_LEFT);
   
  }

  void imageCb_l(const sensor_msgs::ImageConstPtr& msg)
  {
	    cv_bridge::CvImagePtr cv_ptr;
	    try
	    {
	      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    }
	    catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	      return;
	    }

	    // Draw an example circle on the video stream
	    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	      cv::circle(cv_ptr->image, cv::Point(0, 0), 10, CV_RGB(255,0,0));

	  //--------------------------------------------------
      	Mat  my_img(800,800,CV_8UC3,cv::Scalar(0,0,0));
       for(int i=0;i<my_img.rows;i++)
       {
       	for(int j=0;j<my_img.cols;j++)
       	{
       		my_img.at<Vec3b>(i,j)[0]=0;
       		my_img.at<Vec3b>(i,j)[1]=0;
       		my_img.at<Vec3b>(i,j)[2]=0;
       	}
       }

       //	Mat my_img = imread("/home/umesh-cube26-ar/Desktop/mappp.png");

	  //--------------------------------------------------
      /*if(create_flag==0){
   	file_create();
   	create_flag=1;}*/
   	
   	odom(my_img);
   	path_gen(my_img);
   
    
start = clock();
cv::line(my_img,Point(k,h),Point(k-(car_len*sin(alpha)),h-(car_len*cos(alpha))),cv::Scalar(255,0,0),2);



	
	 vel =velocity(xi,yi,h,k); 
	 if(vel<nvel && vel_flag==1)
	 	{cv::putText(my_img, "brake", Point(k+5,h), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,255,255), 1,1);}
	 if(vel<nvel && vel_flag==0)
	 	{cv::putText(my_img, "throttle", Point(k+5,h), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,255,255), 1,1);}
	 
	  h=h-(vel*msecs*cos(alpha));
	  k=k-(vel*msecs*sin(alpha));

	  

end = clock();
	
msecs = ((double) (end-start))*1000/CLOCKS_PER_SEC;
	imshow("imae",my_img);
		if(waitKey(30)>0)
		 { exit(0);}

   	setMouseCallback("imae", CallBackFunc, NULL);
	  
	// search(); 
	  }

};


int main(int argc, char** argv)
{
	  ros::init(argc, argv, "image_converter");
	  ImageConverter ic;

	  ros::spin();
	  return 0;
}