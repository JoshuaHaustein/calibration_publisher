#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

struct passwd *pw = getpwuid(getuid()); // Get the user's home directory
std::string home_dir(pw->pw_dir);

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibration_publisher");
    ros::NodeHandle n("~");
    YAML::Node params_node;
    std::string calibration_file, camera_frame;
    std::vector<double> pose(7);
    
    calibration_file = home_dir + "/.ros/camera_info" + ros::names::parentNamespace(n.getNamespace()) + "_extrinsics.yaml";
    std::cout << "Namespace " << ros::names::parentNamespace(n.getNamespace()) << std::endl;
    std::cout << "Calibration file "<< calibration_file << std::endl;

    // std::ifstream file_in;
    // file_in.open(calibration_file.c_str());
    
    params_node = YAML::LoadFile(calibration_file);
    
    if (!n.getParam("camera_frame", camera_frame))
    {
      ROS_ERROR("No camera frame parameter was passed to the calibration publisher");
      return -1;
    }
    
    for (auto i = 0; i < params_node["pose"].size(); i++)
    {
      pose[i] = params_node["pose"][i].as<double>();
    }
    
    std::cout << "yaml: " << params_node << std::endl;
    std::cout << "pose: " << params_node["pose"] << std::endl;
    std::cout << "base_frame: " << params_node["base_frame"] << std::endl;
    
    tf::TransformBroadcaster br;
    tf::Transform transform;
    
    // initialize transform with identity
    transform.setOrigin(tf::Vector3(pose[0], pose[1], pose[2]) );
    tf::Quaternion q(pose[3], pose[4], pose[5], pose[6]);
    transform.setRotation(q);
    ros::Rate rate(50.0f);
    
    while (n.ok()) {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), params_node["base_frame"].as<std::string>(), camera_frame));
        rate.sleep();
    }
    
    return 0;
}
