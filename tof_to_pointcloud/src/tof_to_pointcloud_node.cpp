#include  "../include/tof_to_pointcloud.hpp"

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "tof_to_pointcloud_node");
	
	ros::NodeHandle nh;
    Tof_to_pointcloud tof_to_pointcloud(nh);


	ros::Rate r(10);
    

	while(ros::ok())
	{
		tof_to_pointcloud.spinner();
		r.sleep();
	}

	return 0;

}
