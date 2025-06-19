#include <haptiquad_contacts/contact_estimator.hpp>


ContactEstimator::ContactEstimator() : Node("haptiquad_contacts"), tf_buffer_(get_clock())
{

    rclcpp::QoS mujoco_qos{rclcpp::SensorDataQoS()};
    mujoco_qos.keep_last(1);    
    
    use_gt = this->declare_parameter<bool>("use_gt", false);
    num_lines = this->declare_parameter<int>("num_lines", 700);
    tolerance = this->declare_parameter<double>("tolerance", 30);
    lines_abs_limit = this->declare_parameter<double>("lines_abs_limit", 1.0);
    force_tol = this->declare_parameter<double>("force_tol", 0.5);
    minimum_force_norm = this->declare_parameter<double>("minimum_force_norm", 0.0);
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization/estimated_contact", 10);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    if (use_gt) {

        gt_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/simulation/base_wrench", mujoco_qos,
            std::bind(&ContactEstimator::gt_callback, this, std::placeholders::_1));

    } else {

        wrench_sub_ = this->create_subscription<haptiquad_msgs::msg::EstimatedForces>(
            "/haptiquad_ros2/estimated_forces", 10,
            std::bind(&ContactEstimator::callback, this, std::placeholders::_1));

    }        

    mesh_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/visualization/collision_model", 10,
        std::bind(&ContactEstimator::mesh_callback, this, std::placeholders::_1));

    
}




void ContactEstimator::mesh_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
    if (!mesh_loaded_)
    {
        std::string uri = msg->mesh_resource;
        if (uri.rfind("package://", 0) == 0)
        {
            uri = uri.substr(10);
            std::string package = uri.substr(0, uri.find("/"));
            std::string relative_path = uri.substr(uri.find("/") + 1);
            std::string full_path = ament_index_cpp::get_package_share_directory(package) + "/" + relative_path;
            std::ifstream input(full_path);

            mesh_ = std::make_shared<open3d::t::geometry::TriangleMesh>();
            *mesh_ = open3d::t::geometry::TriangleMesh::FromLegacy(*open3d::io::CreateMeshFromFile(full_path));

            if (mesh_->IsEmpty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load mesh at %s", full_path.c_str());
                return;
            }

            mesh_->ComputeTriangleNormals();


            mesh_id_ = scene_.AddTriangles(*mesh_);
                        
            mesh_loaded_ = true;
            RCLCPP_INFO(this->get_logger(), "Mesh loaded from %s", full_path.c_str());
        }
    }
}


std::pair<Eigen::Vector3d, Eigen::Vector3d> ContactEstimator::getBaseFT(const geometry_msgs::msg::Wrench wrench, rclcpp::Time stamp)
{

    if (tf_buffer_.canTransform("base", "world", stamp, std::chrono::milliseconds(100))) {

        tf2::Vector3 force_base;
        tf2::Vector3 torque_base;

        geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_.lookupTransform("base", "world", stamp, std::chrono::milliseconds(100));
        tf2::Quaternion q(
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w);
        tf2::Matrix3x3 rotation(q);


        tf2::Vector3 force_world(wrench.force.x, wrench.force.y, wrench.force.z);
        tf2::Vector3 torque_world(wrench.torque.x, wrench.torque.y, wrench.torque.z);
        force_base = rotation * force_world;
        torque_base = rotation * torque_world;

        Eigen::Vector3d force(force_base.x(), force_base.y(), force_base.z());
        Eigen::Vector3d torque(torque_base.x(), torque_base.y(), torque_base.z());
        return {force, torque};

    } else {
        RCLCPP_WARN(this->get_logger(), "Transform not available from world to base");
        return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    }

}



void ContactEstimator::callback(const haptiquad_msgs::msg::EstimatedForces::SharedPtr msg)
{

    if (!mesh_loaded_)
        return;

    auto [force, torque] = getBaseFT(msg->forces[4], msg->header.stamp);

    if (force.norm() < previous_force_norm + force_tol && force.norm() >= previous_force_norm - force_tol) {
        RCLCPP_DEBUG(this->get_logger(), "The force is still the same.");
        publish_marker(last_best_point);
        return;        
    }

    auto [best_point, error] = find_point(force, torque, num_lines);

    if (error < tolerance)
    {
        publish_marker(best_point);
        previous_force_norm = force.norm();
        last_best_point = best_point;
        RCLCPP_DEBUG(this->get_logger(), "Found point [%f, %f, %f] with error %f", best_point.x(), best_point.y(), best_point.z(), error);

    } else {

        publish_marker(last_best_point);

    }

}


void ContactEstimator::gt_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{

    if (!mesh_loaded_)
        return;

    auto [force, torque] = getBaseFT(msg->wrench, msg->header.stamp);

    if (force.norm() < previous_force_norm + force_tol && force.norm() >= previous_force_norm - force_tol) {
        RCLCPP_DEBUG(this->get_logger(), "The force is still the same.");
        publish_marker(last_best_point);
        return;        
    }

    auto [best_point, error] = find_point(force, torque, num_lines);

    if (error < tolerance)
    {
        publish_marker(best_point);
        previous_force_norm = force.norm();
        last_best_point = best_point;
        RCLCPP_DEBUG(this->get_logger(), "Found point [%f, %f, %f] with error %f", best_point.x(), best_point.y(), best_point.z(), error);

    } else {

        publish_marker(last_best_point);

    }


}




std::pair<Eigen::Vector3d, double> ContactEstimator::find_point(
    const Eigen::Vector3d& force,
    const Eigen::Vector3d& torque,
    int lines)
{
    double force_norm = force.norm();
    bool all_line_rejected = true;

    if (force_norm < minimum_force_norm)
    {
        RCLCPP_DEBUG(this->get_logger(), "Force too smal to define direction.");
        return {Eigen::Vector3d::Zero(), 0.0};
    }

    Eigen::Vector3d direction = force / force_norm;
    Eigen::Vector3d point_on_line = force.cross(torque) / (force_norm * force_norm);

    Eigen::Vector3d best_point = Eigen::Vector3d::Zero();
    double best_error = std::numeric_limits<double>::infinity();

    open3d::core::Device device("CPU:0");
    open3d::core::Tensor rays({lines, 6}, open3d::core::Dtype::Float32, device);

    float* rays_ptr = rays.GetDataPtr<float>();
    for (int i = 0; i < lines; ++i)
    {
        double l = -1.0 + lines_abs_limit * i / static_cast<double>(lines - 1);
        Eigen::Vector3d origin = point_on_line;
        Eigen::Vector3d dir = direction * l;

        for (int j = 0; j < 3; ++j)
        {
            rays_ptr[i * 6 + j]     = static_cast<float>(origin[j]);
            rays_ptr[i * 6 + j + 3] = static_cast<float>(dir[j]);
        }
    }

    auto ans = scene_.CastRays(rays);
    auto& t_hit_tensor = ans.at("t_hit");
    auto& geometry_ids = ans.at("geometry_ids");
    auto& primitive_normals = ans.at("primitive_normals"); 

    const float* t_hit = t_hit_tensor.GetDataPtr<float>();
    const uint32_t* geom_ids = geometry_ids.GetDataPtr<uint32_t>();
    const float* normals_ptr = primitive_normals.GetDataPtr<float>();


    for (int i = 0; i < lines; ++i)
    {
        if (t_hit[i] > 0.0f && geom_ids[i] != -1)
        {
            Eigen::Vector3d origin(
                rays_ptr[i * 6 + 0],
                rays_ptr[i * 6 + 1],
                rays_ptr[i * 6 + 2]);

            Eigen::Vector3d dir(
                rays_ptr[i * 6 + 3],
                rays_ptr[i * 6 + 4],
                rays_ptr[i * 6 + 5]);

            Eigen::Vector3d normal(
                normals_ptr[i * 3 + 0],
                normals_ptr[i * 3 + 1],
                normals_ptr[i * 3 + 2]
            );


            // if (force.transpose() * normal >= 0) {
            //     // RCLCPP_ERROR_STREAM(this->get_logger(), force.dot(normal));
            //     continue;
            // }
            all_line_rejected = false;


            Eigen::Vector3d point = origin + t_hit[i] * dir;

            Eigen::Vector3d expected_torque = point.cross(force);
            double error = (expected_torque - torque).norm();

            if (error < best_error)
            {
                best_error = error;
                best_point = point;

            }
        }
    }

    if (all_line_rejected) {
        RCLCPP_WARN_STREAM(this->get_logger(), "No hit with geometry");
    }

    return {best_point, best_error};
}



void ContactEstimator::publish_marker(const Eigen::Vector3d& position)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base";
    marker.header.stamp = this->now();
    marker.ns = "estimated";
    marker.id = 20;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration(0, 0);

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 0.8;
    marker.color.r = 0.3;
    marker.color.g = 0.5;
    marker.color.b = 0.1;

    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;

    marker_pub_->publish(marker);
}




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContactEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}