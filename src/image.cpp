#include <stdio.h>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"

using namespace ros;
using namespace cv;

const cv::Scalar BLACK = cv::Scalar(0,0,0);
const cv::Scalar RED = cv::Scalar(0,0,255);
const cv::Scalar BLUE = cv::Scalar(255,0,0);
const cv::Scalar GREEN = cv::Scalar(0,255,0);

class timer
{
private:
	float begin;
	float end;

public:
	timer();
	~timer();
	void start();
	void finish();
	float diff();
};

timer::timer() {}

timer::~timer() {}

void timer::start()
{
	begin = clock();
}

void timer::finish()
{
	end = clock();
}

float timer::diff()
{
	return ((end-begin)/CLOCKS_PER_SEC);
}

class image
{
public:
	image();
	void getImage();
	void colour(cv::Mat oriImg);
	void createHull(cv::Mat thImg);
	void imageCallback(const sensor_msgs::Image::ConstPtr& image);
	void controlRobot(double conArea, Point2f centroid);

private:
	NodeHandle n;
	Subscriber sub;
	Publisher pub;
	int imageHolder;
};

image::image()
{
	sub = n.subscribe("/camera/depth/image_raw", 1000, &image::imageCallback, this);
	pub = n.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 10);
}

void image::getImage()
{
	spinOnce();
}

void image::controlRobot(double conArea, Point2f centroid)
{
	geometry_msgs::Twist msg;

	ROS_INFO_STREAM("Controlling Robot now!!!");

	if ((conArea > 7500) && (conArea < 10000))
		msg.linear.x = 0.0;
	else
		msg.linear.x = 0.5;

	if ((centroid.x) < 100)
		msg.angular.z = -0.5;
	else if ((centroid.x) > 540)
		msg.angular.z = 0.5;
	else if ((centroid.y) < 100)
		msg.linear.x = msg.linear.x + 0.3;
	else if ((centroid.y) > 380)
		msg.linear.x = msg.linear.x - 0.1;

	pub.publish(msg);
}

void image::createHull(cv::Mat thImg)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(thImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<vector<Point> >hullPoints(contours.size());
	vector<Moments> mu(contours.size());
	vector<Point2f> mc(contours.size());
	for (int i=0; i<contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hullPoints[i], false);
		mu[i] = moments(contours[i], false);	
		mc[i] = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);	
	}

	Mat drawing = Mat::zeros(thImg.size(), CV_8UC3);
	for (int i=0; i<contours.size(); i++)
	{
		if ((contourArea(contours[i]) > 7500) && (contourArea(contours[i]) < 20000))
		{
			drawContours(drawing, contours, i, RED, 1, 8, vector<Vec4i>(), 0, Point());
			drawContours(drawing, hullPoints, i, GREEN, 1, 8, vector<Vec4i>(), 0, Point());
			circle(drawing, mc[i], 4, BLUE, -1, 8, 0);
			this->controlRobot(contourArea(contours[i]), mc[i]);
		}
	}

	imshow("Hull Demo", drawing);
}

void image::colour(cv::Mat oriImg)
{
	cv::Mat threshold(oriImg.rows, oriImg.cols, CV_8UC1, cv::Scalar(0));

	for (int i=0; i<oriImg.rows; i++)
	{
		for (int j=0; j<oriImg.cols; j++)
		{
			imageHolder = oriImg.at<ushort>(i,j);

			switch (imageHolder>>8)
			{
				case 1:
				case 2:
					threshold.at<uchar>(i,j) = 255;
					break;
				default:
					break;
			}
		}
	}
	this->createHull(threshold);
	//cv::imshow("Image", threshold);
}

void image::imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
	//ROS_INFO_STREAM("Getting a callback... ");
	//ROS_INFO_STREAM("GOT IMAGE!!! " << image->height << "   "  << image->width << "      "  << image->step << "    "  << image->encoding);
	
	int size = (image->height)*(image->width)*2;
	uint8_t depth_data[size];

	for (int i=0; i<size; i++)
		depth_data[i] = image->data[i];

	cv::Mat disparity(image->height, image->width, CV_16UC1, depth_data, (image->width)*2);

	//disparity.convertTo(disparity, CV_8UC1);
	//imshow("Disparity", disparity);
	this->colour(disparity);	
}

int main(int argc, char** argv)
{
	init(argc, argv, "gettingImage");
	image Img;

	Rate rate_loop(1000);

	while((ok()) && (cv::waitKey(1) < 0))
	{
		Img.getImage();
		rate_loop.sleep();
	}

	return 0;
}