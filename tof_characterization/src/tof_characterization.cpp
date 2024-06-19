#include "../include/tof_characterization.hpp"

Tof_characterization::Tof_characterization(ros::NodeHandle &nh)
{
    this->nh = nh;
    nh.getParam("/tof_characterization_node/num_measurements", num_measurements);
    ROS_INFO("Number of measurements: %d", num_measurements);
    nh.getParam("/tof_characterization_node/distance", distance);
    ROS_INFO("Distance: %f", distance);

    sub_tof_acquisition = nh.subscribe("/tof_pointcloud", 1, &Tof_characterization::store_pointcloud, this);
    tof_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    wait_for_service();
    client_generate_tof_map = nh.serviceClient<generate_tof_map::GenerateHeatmap>("generate_heatmap");

    // inizialization
    actual_num_measurements = 0;
    assolute_average.resize(num_points);
    point_deviation.resize(num_points);
    squared_deviation_sum.resize(num_points);
    abs_std.resize(num_points);
    mean_error.resize(num_points);
    ass_mean_error.resize(num_points);
    variance.resize(num_points);
}

void Tof_characterization::store_pointcloud(const sensor_msgs::PointCloud2 &msg)
{
    ROS_INFO("entered in callback, actual measurement: %d", actual_num_measurements);
    pcl::fromROSMsg(msg, *tof_pointcloud);

    if (tof_pointcloud->points.empty())
    {
        ROS_ERROR("Received empty point cloud");
        return;
    }

    ROS_INFO("Pointcloud received");

    actual_num_measurements++;

    if (actual_num_measurements == 1)
    {
        characterization_routine_single_measurement();
    }
    else
    {
        characterization_routine_multiple_measurement();
    }

    if (actual_num_measurements == num_measurements)
    {
        std::string name_graph = "absolute_std_dev table color:";
        generate_heatmap_service(abs_std, name_graph);
        name_graph = "absolute_mean_error table color:";
        generate_heatmap_service(ass_mean_error, name_graph);
        name_graph = "avarage:";
        generate_heatmap_service(assolute_average, name_graph);
    }
}

void Tof_characterization::characterization_routine_single_measurement()
{
    ROS_INFO("entered in single measurement, actual measurement: %d", actual_num_measurements);
    calculate_average_measurement();
    calculate_std_deviation();
    calculate_mean_error();
}

void Tof_characterization::characterization_routine_multiple_measurement()
{
    calculate_average_measurement();
    calculate_ass_mean_error();
    calculate_ass_std();
}

void Tof_characterization::calculate_average_measurement(void)
{
    for (int i = 0; i < num_points; i++)
    {
        assolute_average[i] = ((assolute_average[i] * (actual_num_measurements - 1)) + tof_pointcloud->points[i].z) / actual_num_measurements;
    }
}

void Tof_characterization::calculate_std_deviation(void)
{
    // Std deviation of a sigle measurement
    for (size_t i = 0; i < num_points; ++i)
    {
        point_deviation[i] = std::abs(tof_pointcloud->points[i].z - assolute_average[i]);
    }
}

void Tof_characterization::calculate_ass_std(void)
{
    calculate_std_deviation();

    for (size_t i = 0; i < num_points; ++i)
    {
        squared_deviation_sum[i] += point_deviation[i] * point_deviation[i];
    }

    for (size_t i = 0; i < num_points; ++i)
    {
        variance[i] = squared_deviation_sum[i] / actual_num_measurements;
        abs_std[i] = std::sqrt(variance[i]);
    }
}

void Tof_characterization::calculate_mean_error(void)
{

    for (size_t i = 0; i < num_points; ++i)
    {
        mean_error[i] = std::abs(tof_pointcloud->points[i].z - distance);
    }
}

void Tof_characterization::calculate_ass_mean_error(void)
{
    calculate_mean_error();

    for (size_t i = 0; i < num_points; ++i)
    {
        ass_mean_error[i] = ((ass_mean_error[i] * (actual_num_measurements - 1)) + mean_error[i]) / actual_num_measurements;
    }
}

void Tof_characterization::generate_heatmap_service(const std::vector<double> &data, const std::string &name_graph)
{
    generate_tof_map::GenerateHeatmap srv;

    // Convert the 2D vector to a flat array

    srv.request.data = data;

    srv.request.rows = 8;
    srv.request.cols = 8;
    srv.request.name_graph = name_graph;

    if (client_generate_tof_map.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("Heatmap saved successfully at: %s", srv.response.message.c_str());
        }
        else
        {
            ROS_ERROR("Failed to generate heatmap: %s", srv.response.message.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service generate_heatmap");
    }
}

// void Tof_characterization::generate_heatmap_service(const std::vector<double> &data1, const std::string &name_graph, const std::vector<double> *data2 = nullptr)
// {
//     generate_tof_map::GenerateHeatmap srv;

//     std::vector<double> combined_data;

//     if (data2 != nullptr)
//     {
//         if (data1.size() != data2->size())
//         {
//             ROS_ERROR("Data vectors must be of the same size");
//             return;
//         }

//         const int num_elements = data1.size();
//         combined_data.reserve(num_elements * 2);
//         for (size_t i = 0; i < num_elements; ++i)
//         {
//             combined_data.push_back(data1[i]);
//             combined_data.push_back((*data2)[i]);
//         }
//     }
//     else
//     {
//         combined_data = data1;
//     }

//     srv.request.data = combined_data;
//     srv.request.rows = 8; // Modifica secondo necessità
//     srv.request.cols = 8; // Modifica secondo necessità
//     srv.request.name_graph = name_graph;

//     if (client_generate_tof_map.call(srv))
//     {
//         if (srv.response.success)
//         {
//             ROS_INFO("Heatmap saved successfully at: %s", srv.response.message.c_str());
//         }
//         else
//         {
//             ROS_ERROR("Failed to generate heatmap: %s", srv.response.message.c_str());
//         }
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service generate_heatmap");
//     }
// }

void Tof_characterization::generate_heatmap_service(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &name_graph)
{
    generate_tof_map::GenerateHeatmap srv;
    std::vector<double> data;

    // Convert the point cloud to a flat array containing only z values
    for (const auto &point : cloud->points)
    {
        data.push_back(point.z);
    }

    srv.request.data = data;

    srv.request.rows = 8;
    srv.request.cols = 8;
    srv.request.name_graph = name_graph;

    if (client_generate_tof_map.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("Heatmap saved successfully at: %s", srv.response.message.c_str());
        }
        else
        {
            ROS_ERROR("Failed to generate heatmap: %s", srv.response.message.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service generate_heatmap");
    }
}

void Tof_characterization::wait_for_service(void)
{
    ROS_INFO("Waiting for generate_heatmap service to be available...");
    ros::service::waitForService("generate_heatmap");
    ROS_INFO("Service generate_heatmap is now available.");
}
void Tof_characterization::spinner()
{
    ros::spinOnce();
}
