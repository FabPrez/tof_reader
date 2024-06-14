#include  "../include/tof_characterization.hpp"

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "tof_characterization_node");
	
	ros::NodeHandle nh;
    Tof_characterization Tof_characterization(nh);
	ros::Rate r(10);
    
	std::vector<std::vector<double>> data = {
        {42.06, 33.41, 28.50, 25.37, 24.31, 23.74, 24.88, 30.62},
        {34.03, 29.05, 27.60, 25.27, 23.77, 22.45, 20.76, 23.08},
        {29.78, 27.86, 26.75, 27.54, 26.17, 24.43, 20.07, 21.20},
        {29.30, 27.05, 28.53, 29.53, 29.38, 23.96, 21.88, 21.12},
        {28.65, 27.17, 28.71, 29.01, 27.08, 24.20, 21.61, 19.31},
        {30.92, 26.03, 27.43, 26.71, 25.71, 23.86, 19.90, 20.62},
        {33.69, 28.46, 24.07, 24.55, 23.70, 22.65, 20.73, 26.06},
        {39.72, 32.02, 26.41, 24.82, 24.90, 23.87, 25.58, 33.10}};

    std::string name_graph = "aooooo";
    Tof_characterization.generate_heatmap_service(data, name_graph);

	while(ros::ok())
	{
		Tof_characterization.spinner();
		r.sleep();
	}

	return 0;

}
