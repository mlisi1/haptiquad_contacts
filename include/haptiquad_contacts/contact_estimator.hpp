#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <haptiquad_msgs/msg/estimated_forces.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <open3d/Open3D.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/t/geometry/RaycastingScene.h>


class ContactEstimator : public rclcpp::Node
{
    public:

        ContactEstimator();

        void mesh_callback(const visualization_msgs::msg::Marker::SharedPtr msg);
        void callback(const haptiquad_msgs::msg::EstimatedForces::SharedPtr msg);
        void gt_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void publish_marker(const Eigen::Vector3d& position);


        std::pair<Eigen::Vector3d, Eigen::Vector3d> getBaseFT(const geometry_msgs::msg::Wrench wrench, rclcpp::Time stamp);
        std::pair<Eigen::Vector3d, double> find_point(const Eigen::Vector3d& force, const Eigen::Vector3d& torque, int lines);

        
    private:
        
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Subscription<haptiquad_msgs::msg::EstimatedForces>::SharedPtr wrench_sub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr gt_wrench_sub_;
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr mesh_sub_;

        tf2_ros::Buffer tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::shared_ptr<open3d::t::geometry::TriangleMesh> mesh_;
        open3d::t::geometry::RaycastingScene scene_;
        uint32_t mesh_id_;


        double tolerance;
        int num_lines;
        double lines_abs_limit;
        double previous_force_norm = 0.0;
        double force_tol;
        bool use_gt;
        double minimum_force_norm;

        Eigen::Vector3d last_best_point = Eigen::Vector3d::Zero();
        bool mesh_loaded_ = false;

};