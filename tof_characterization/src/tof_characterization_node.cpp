#include  "../include/tof_characterization.hpp"

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "Tof_characterization_node");
	
	ros::NodeHandle nh;
    Tof_characterization Tof_characterization(nh);


	ros::Rate r(10);
    

	while(ros::ok())
	{
		Tof_characterization.spinner();
		r.sleep();
	}

	return 0;

}
