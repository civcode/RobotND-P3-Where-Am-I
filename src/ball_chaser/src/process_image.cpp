#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sstream>

ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)  {
    
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
	ROS_ERROR("Failed to call servivce command_robot");
}


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::ImageConstPtr& img) {

    int white_pixel = 255;
	cv_bridge::CvImagePtr cv_ptr;
	try {
	    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
		//cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
	} catch (cv_bridge::Exception& e) {
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	}

	cv::Mat img_gray(cv_ptr->image.size(), CV_8U);
	cv::cvtColor(cv_ptr->image, img_gray, CV_BGR2GRAY);

	cv::Mat img_binary(img_gray.size(), img_gray.type());

	cv::threshold(img_gray, img_binary, 254, 255, cv::THRESH_BINARY);

	cv::Point text_offset(10, 30);
	// Verify that binary image is not all zeros
	if (cv::sum(img_binary).val[0] > 0) {
		cv::Moments mu;

		mu = cv::moments(img_binary, true);

		float x_mean = static_cast<float>(mu.m10/mu.m00);
		float y_mean = static_cast<float>(mu.m01/mu.m00);

		std::ostringstream oss;
		oss.precision(2);
		oss << "cols:" << img_binary.cols << ", rows:"  << img_binary.rows << ", mean[" 
			<< x_mean << "," << y_mean << "]";

		cv::putText(img_binary, oss.str(), text_offset, cv::FONT_HERSHEY_DUPLEX, 1.0, 
					CV_RGB(255,255,255), 2);

		printf("mean: %1.2f, %1.2f\n", x_mean, y_mean);

	} else {
		std::ostringstream oss;
		oss.precision(2);
        oss.setf(std::ios::fixed);
		oss << "cols:" << img_binary.cols << ", rows:"  << img_binary.rows << ", mean[na,na]"; 

		cv::putText(img_binary, oss.str(), text_offset, cv::FONT_HERSHEY_DUPLEX, 1.0, 
					CV_RGB(255,255,255), 2);

	}

	//cv::imshow("grayscale image", img_gray);
	cv::imshow("binary image", img_binary);

	cv::waitKey(1);
            
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	//cv::namedWindow("grayscale image", cv::WINDOW_NORMAL);
    cv::namedWindow("binary image", cv::WINDOW_NORMAL);

	ROS_INFO("Ready to process images");

	ros::spin();

    return 0;
}
