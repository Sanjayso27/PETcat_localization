#include <depth_headers/depth.hpp>

namespace roboiitk::depth_map{

   void depth_map::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private){
        image_sub = nh.subscribe("disparity_img", 1, &depth_map::imageCb, this);
        depthmap_pub= nh.advertise<sensor_msgs::Image>("depth_map", 1);
   }
   void depth_map::imageCb(const stereo_msgs::DisparityImage& msg) {
        cv_bridge::CvImagePtr cv_ptr;
	    try {cv_ptr = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::TYPE_32FC1);}
        catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what()); }
        cv::Mat frame;
        frame = cv_ptr->image;
        ROS_ASSERT(frame.empty() != true);
        cv::Mat depth = cv::Mat::zeros(frame.size(), CV_32F);
        for( int i = 0; i < frame.rows; ++i)
            for( int j = 0; j < frame.cols; ++j ){
                if(int(frame.at<unsigned char>(i,j))!=0){
                    depth.at<float>(i,j)=float((320.25491333* 0.070000000298)/(int(frame.at<unsigned char>(i,j))));
                    if(depth.at<float>(i,j)>1.00){ROS_INFO("yo");std::cout<<depth.at<float>(i,j)<<int(frame.at<unsigned char>(i,j));}
                }
        }
        cv_bridge::CvImage depth_image;
		depth_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		depth_image.header.stamp = ros::Time::now();
		depth_image.image = depth;
		depthmap_pub.publish(depth_image.toImageMsg());
   }
   void depth_map::run() {
   } 
}