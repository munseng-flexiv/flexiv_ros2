#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <string>
#include <fstream>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_waypoints");

void read_csv(std::string filename, std::vector<std::vector<double>> &data)
{
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(lineStream, cell, ' '))
        {
            row.push_back(std::stod(cell));
        }
        data.push_back(row);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_waypoints", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "rizon_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // Getting Basic Information
    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Planning to a Joint Goal
    std::vector<double> joint_group_positions;
    move_group.getCurrentState()->copyJointGroupPositions(PLANNING_GROUP, joint_group_positions);
    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = - 40.0 * M_PI / 180.0;
    joint_group_positions[2] = 0.0;
    joint_group_positions[3] = 90.0 * M_PI / 180.0;
    joint_group_positions[4] = 0.0;
    joint_group_positions[5] = 40.0 * M_PI / 180.0;
    joint_group_positions[6] = 0.0;
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (joint goal) %s", success ? "" : "FAILED");

    // Execute the plan
    move_group.move();

    // Cartesian Paths
    std::vector<std::vector<double>> data;
    std::string csv_path = "";
    read_csv(csv_path, data);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (int i = 0; i < data.size(); i++)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = data[i][0];
        pose.position.y = data[i][1];
        pose.position.z = data[i][2];
        pose.orientation.w = data[i][3];
        pose.orientation.x = data[i][4];
        pose.orientation.y = data[i][5];
        pose.orientation.z = data[i][6];
        waypoints.push_back(pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory);
}
