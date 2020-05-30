#include <depth_headers/depth.hpp>

using namespace roboiitk::depth_map;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"depth_map_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    depth_map map;

    map.init(nh,nh_private);

    ros::Rate loopRate(1);

    while(ros::ok())
    {
        map.run();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}