#pragma once

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

namespace roboiitk::depth_map {
class depth_map {
    private:
    ros::Subscriber image_sub;
    ros::Publisher depthmap_pub;

    public:
	void imageCb(const stereo_msgs::DisparityImage& msg);
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
	void run();
    };
} // namespace roboiitk::depth_map