#include "ex6/potential_field_controller.hpp"
#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <urdf/model.h>
#include <custom_msgs/msg/obstacle_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace ex6 {


controller_interface::return_type PotentialFieldController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
    // Update joint states from hardware state interface
    updateJointStates();

    elapsed_time_ = elapsed_time_ + period.seconds();

    // Save current state to KDL variables for solvers
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_kdl_(i) = position_interface_vals_(i);
        q_dot_kdl_(i) = velocity_interface_vals_(i);
    }

    // Read the goal pose
    xd_kdl_ = *rt_goal_pose_buffer_.readFromRT()->get();

    /**
     * TODO: implement attractive force/velocity
     */
    // Forward kinematics
    KDL::Frame x_kdl;
    solver_->computeFK(q_kdl_, x_kdl);

    // Jacobian
    KDL::Jacobian J_kdl(NUM_JOINTS);
    solver_->compute_jac(q_kdl_, J_kdl);

    // Error in task space
    KDL::Twist xe_kdl = KDL::diff(x_kdl, xd_kdl_);

    // Convert error to Eigen vector
    Eigen::Matrix<double, 6, 1> xe;

    xe << xe_kdl.vel.x(),
          xe_kdl.vel.y(),
          xe_kdl.vel.z(),
          xe_kdl.rot.x(),
          xe_kdl.rot.y(),
          xe_kdl.rot.z();

    // Proportional gain
    Eigen::Matrix<double, 6, 1> Kp;
    Kp.setConstant(1.0);

    // Desired end-effector velocity
    Eigen::Matrix<double, 6, 1> xd_dot = Kp.cwiseProduct(xe);

    // Jacobian pseudoinverse
    Eigen::Matrix<double, 6, 7> J = J_kdl.data;
    Eigen::Matrix<double, 7, 6> J_pinv;

    solver_->get_damped_pseudo_inverse(J, J_pinv);

    // Attractive joint velocity
    Vector7d q_dot_att = J_pinv * xd_dot;


    /**
     * TODO: implement joint limits as APF
     */

    Vector7d q_dot_rep = Vector7d::Zero();

    double k = 0.00001;
    double threshold = 0.3;
    double Q = threshold * threshold;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {

        double q = position_interface_vals_(i);

        // Minimum joint limit
        double dist_min = std::max(q - joint_min_(i), 0.01);
        double D_min = dist_min * dist_min;

        if (dist_min < threshold) {
            double F_min = k * (1.0 / D_min - 1.0 / Q)
                             * (1.0 / (D_min * D_min))
                             * (2.0 * dist_min);

            q_dot_rep(i) += F_min;
        }

        // Maximum joint limit
        double dist_max = std::max(joint_max_(i) - q, 0.01);
        double D_max = dist_max * dist_max;

        if (dist_max < threshold) {
            double F_max = k * (1.0 / D_max - 1.0 / Q)
                             * (1.0 / (D_max * D_max))
                             * (-2.0 * dist_max);

            q_dot_rep(i) += F_max;
        }

    // Limit the repulsive joint velocity
    q_dot_rep(i) = std::clamp(q_dot_rep(i), -0.6, 0.6);
}


    /**
     * TODO: implement obstacle avoidance with APF
     */
    // Read the obstacle info
    const auto& obstacles = *rt_obstacles_.readFromRT();
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        KDL::Frame x_kdl_inner;
        solver_->computeFK(q_kdl_, x_kdl_inner, i);

        for (const auto& obs : obstacles) {
            Eigen::Vector3d p_obs(obs.position.x, obs.position.y, obs.position.z);

            // Spherical obstacle
            if (obs.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
                // Sphere radius
                double obs_r = obs.radius;
                /**
                 * TODO: spherical object avoidance
                 */

                // Current end-effector position
                Eigen::Vector3d p(
                    x_kdl_inner.p.x(),
                    x_kdl_inner.p.y(),
                    x_kdl_inner.p.z()
                );

                // Difference between end effector and obstacle
                Eigen::Vector3d diff = p - p_obs;

                // Squared distance
                double D = std::max(diff.squaredNorm(), 0.0001);

                // Obstacle influence threshold
                double threshold = obs_r + 0.2;
                double Q = threshold * threshold;

                // Repulsive gain
                double k_sphere = 0.0001;

                if (D < Q) {

                    // Gradient of distance with respect to joints
                    Eigen::Matrix<double, 1, 7> dD_dq =
                        2.0 * diff.transpose() * J.topRows<3>();

                    // Calculate repulsive joint velocity
                    Vector7d F_rep =
                        k_sphere * (1.0 / D - 1.0 / Q)
                        * (1.0 / (D * D))
                        * dD_dq.transpose();

                    // Add to total repulsive velocity
                    q_dot_rep += F_rep;
                }
            }
            // Box obstacle (such as wall)
            else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
                // Box dimensions vector
                Eigen::Vector3d box_size(obs.box_size.at(0), obs.box_size.at(1), obs.box_size.at(2));
                /**
                 * TODO: box obstacle avoidance
                 */
                // Current end-effector position
                Eigen::Vector3d p(
                    x_kdl_inner.p.x(),
                    x_kdl_inner.p.y(),
                    x_kdl_inner.p.z()
                );

                // Box orientation
                Eigen::Quaterniond box_q(
                    obs.orientation.w,
                    obs.orientation.x,
                    obs.orientation.y,
                    obs.orientation.z
                );

                if (box_q.squaredNorm() < 1e-12) {
                    box_q = Eigen::Quaterniond::Identity();
                } else {
                    box_q.normalize();
                }

                // Transform end-effector position into box coordinates
                Eigen::Vector3d p_local = box_q.inverse() * (p - p_obs);

                // Half dimensions of box
                Eigen::Vector3d half = box_size / 2.0;

                // Closest point on or inside box
                Eigen::Vector3d closest =
                    p_local.cwiseMax(-half).cwiseMin(half);

                // Vector from closest point to end effector
                Eigen::Vector3d diff = p_local - closest;

                // Distance to box
                double distance = diff.norm();

                // Repulsive direction
                Eigen::Vector3d direction;

                if (distance > 0.000001) {
                    direction = diff / distance;
                }
                else {
                    // End effector is inside or on the box
                    // Find the nearest box face
                    Eigen::Vector3d margins = half - p_local.cwiseAbs();

                    Eigen::Index axis;
                    margins.minCoeff(&axis);

                    direction = Eigen::Vector3d::Zero();

                    if (p_local(axis) >= 0.0) {
                        direction(axis) = 1.0;
                    } else {
                direction(axis) = -1.0;
                }
                }

                // Convert direction back into robot coordinates
                Eigen::Vector3d normal = box_q * direction;

                // Avoid division by zero
                double D = std::max(distance, 0.01);

                // Obstacle influence distance (meters)
                double Q = 0.25;

                // Repulsive gain
                double k_box = 0.001;

                if (D < Q) {

                    // Distance gradient in joint space
                    Eigen::Matrix<double, 1, 7> dD_dq = normal.transpose() * J.topRows<3>();

                    // Repulsive joint velocity
                    Vector7d F_rep = k_box * (1.0 / D - 1.0 / Q) * (1.0 / (D * D)) * dD_dq.transpose();

                    // Add to total repulsive velocity
                    q_dot_rep += F_rep;
                }
            }
        }
    }
    
    
    // Limit total repulsive joint velocity
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_dot_rep(i) = std::clamp(q_dot_rep(i), -0.6, 0.6);
    }

    /**
     * TODO: final torque command
     */
    // Combine attractive and repulsive velocities
    Vector7d q_dot_cmd;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // Limit attractive velocity first
        q_dot_att(i) = std::clamp(q_dot_att(i), -0.3, 0.3);

        q_dot_cmd(i) = q_dot_att(i) + q_dot_rep(i);

        // Prevent further movement toward a very close joint limit
        if (position_interface_vals_(i) - joint_min_(i) < 0.05 &&
            q_dot_cmd(i) < 0.0) {
            q_dot_cmd(i) = 0.0;
        }

        if (joint_max_(i) - position_interface_vals_(i) < 0.05 &&
            q_dot_cmd(i) > 0.0) {
            q_dot_cmd(i) = 0.0;
        }

        q_dot_cmd(i) = std::clamp(q_dot_cmd(i), -0.3, 0.3);
    }

    // Get robot dynamics
    KDL::JntSpaceInertiaMatrix M_kdl(NUM_JOINTS);
    KDL::JntArray G_kdl(NUM_JOINTS);
    KDL::JntArray C_kdl(NUM_JOINTS);

    solver_->compute_dyn_params(
        q_kdl_, q_dot_kdl_, M_kdl, G_kdl, C_kdl
    );

    // Convert KDL data to Eigen
    Eigen::Matrix<double, 7, 7> M = M_kdl.data;

    Vector7d G = G_kdl.data;
    Vector7d C = C_kdl.data;

    // Velocity controller
    Vector7d Kd = Vector7d::Constant(5.0);

    Vector7d e_dot = q_dot_cmd - velocity_interface_vals_;

    Vector7d u = Kd.cwiseProduct(e_dot);

    // Calculate final torque
    Vector7d tau = M * u + G + C;

    // Send torque commands to the hardware command interface
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces_[i].set_value(tau(i));
    }

    publish_markers();

    return controller_interface::return_type::OK;
}

CallbackReturn PotentialFieldController::on_init() {
    // Get parameters from franka_controllers.yaml
    try {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", state_interface_types_);

        root_link_ = auto_declare<std::string>("root_link", root_link_);
        tip_link_ = auto_declare<std::string>("tip_link", tip_link_);
    } 
    catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Robot initialization done.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PotentialFieldController::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration started");

    // Get the robot description parameter (effort_fr3_arm.urdf.xacro parsed into one URDF as a string)
    // Due to AsyncParametersClient, this has to be in on_configure() method.
    auto parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    parameters_client->wait_for_service();
    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();

    // Check if robot description was retrieved succesfully
    if (!result.empty()) {
        robot_description_ = result[0].value_to_string();
    }
    else {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
        return CallbackReturn::ERROR;
    }

    // Parse URDF string into URDF object
    urdf::Model urdf;
    if (!urdf.initString(robot_description_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
        return CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_node()->get_logger(), "Found robot_description");
    }

    // Get the joint angle limits from the URDF
    for (std::size_t i = 0; i < NUM_JOINTS; i++) {
        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf) {
            RCLCPP_ERROR(get_node()->get_logger(), "Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return CallbackReturn::ERROR;
        }
        /**
         * TODO: read the joint limits to some data structure
         */
        if (!joint_urdf->limits) {
            RCLCPP_ERROR(get_node()->get_logger(), "Joint limits not found");
            return CallbackReturn::ERROR;
        }

        joint_min_(i) = joint_urdf->limits->lower;
        joint_max_(i) = joint_urdf->limits->upper;
    }

    // Construct a KDL tree object from the URDF
    if (!kdl_parser::treeFromUrdfModel(urdf, tree_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct a KDL tree.");
        return CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_node()->get_logger(), "Constructed a KDL tree.");
    }

    // Get the KDL chain from KDL tree (for KDL Solver objects)
    if (!tree_.getChain(root_link_, tip_link_, chain_)) {
        kdl_chain_error_print();

        return CallbackReturn::ERROR;
    }
    else {
        kdl_chain_success_print();
    }

    // Create a Solver instance
    solver_.reset(new Solver(chain_));

    // Solvers for whole-body obstacle avoidance
    link_fk_solver_ =
        std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

    link_jac_solver_ =
        std::make_unique<KDL::ChainJntToJacSolver>(chain_);

    // Create a subscription for the joint command
    goal_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", rclcpp::SystemDefaultsQoS(), std::bind(&PotentialFieldController::goal_pose_callback,
        this, std::placeholders::_1));

    init_obstacle_subscriber_ = get_node()->create_subscription<custom_msgs::msg::ObstacleArray>(
        "obstacles", rclcpp::QoS(1).transient_local(), std::bind(&PotentialFieldController::init_obstacle_callback,
        this, std::placeholders::_1));

    sub_obj_pose_ = get_node()->create_subscription<tf2_msgs::msg::TFMessage>(
        "world/obstacle_world/pose/info", rclcpp::SystemDefaultsQoS(),
        std::bind(&PotentialFieldController::object_pose_callback, this, std::placeholders::_1));

    // Publisher
    pub_markers_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 1000);

    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PotentialFieldController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : command_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

controller_interface::InterfaceConfiguration
PotentialFieldController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : state_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

CallbackReturn PotentialFieldController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    updateJointStates();
    elapsed_time_ = 0.0;

    q_kdl_.resize(NUM_JOINTS);
    q_dot_kdl_.resize(NUM_JOINTS);

    // Initialize realtime buffer
    auto xd = std::make_shared<KDL::Frame>();
    xd->p = KDL::Vector(0.5, 0.0, 1.6);
    xd->M = KDL::Rotation::EulerZYX(0.0, 0.0, M_PI);
    rt_goal_pose_buffer_.writeFromNonRT(xd);

    // Activate publisher
    pub_markers_->on_activate();

    return CallbackReturn::SUCCESS;
}

void PotentialFieldController::updateJointStates() {
    // Pre-check array size to avoid bounds checking in loop
    if (state_interfaces_.size() != state_interface_types_.size() * NUM_JOINTS) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces");
        return;
    }

    // Get the current joint positions
    auto* interfaces = state_interfaces_.data();
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // Access interfaces directly with pointer arithmetic
        const auto& position_interface = interfaces[2*i];
        const auto& velocity_interface = interfaces[2*i + 1];

        // Interface name comparison
        const auto& pos_name = position_interface.get_interface_name();
        if (pos_name != "position") {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected position interface, but got %s",
                         pos_name.c_str());
            return;
        }
        const auto& vel_name = velocity_interface.get_interface_name();
        if (vel_name != "velocity") {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected velocity interface, but got %s",
                         vel_name.c_str());
            return;
        }
        
        // Direct value assignment
        position_interface_vals_(i) = position_interface.get_value();
        velocity_interface_vals_(i) = velocity_interface.get_value();
    }
}

// Joint command subscriber callback
void PotentialFieldController::goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
    auto xd = std::make_shared<KDL::Frame>();

    xd->p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    xd->M = KDL::Rotation::Quaternion(
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    rt_goal_pose_buffer_.writeFromNonRT(xd);
}

void PotentialFieldController::init_obstacle_callback(const custom_msgs::msg::ObstacleArray msg) {
    obstacle_map_.clear(); // In case a new scene was loaded, otherwise no new messages will be published
    std::vector<custom_msgs::msg::Obstacle> obs_vec;
    for (auto obs : msg.obstacles) {
        obstacle_map_.emplace(obs.name, obs);
        obs_vec.push_back(obs);
    }

    rt_obstacles_.writeFromNonRT(obs_vec);
}

void PotentialFieldController::object_pose_callback(const tf2_msgs::msg::TFMessage& msg) {
    for (geometry_msgs::msg::TransformStamped t : msg.transforms) {
        auto it = obstacle_map_.find(t.child_frame_id);
        if (it != obstacle_map_.end()) {
            it->second.position.x = t.transform.translation.x;
            it->second.position.y = t.transform.translation.y;
            it->second.position.z = t.transform.translation.z;
            it->second.orientation = t.transform.rotation;
        }
    }

    // Build a fresh snapshot vector to write to rt buffer
    std::vector<custom_msgs::msg::Obstacle> snapshot;
    snapshot.reserve(obstacle_map_.size());
    for (const auto& [name, obs] : obstacle_map_) {
        snapshot.push_back(obs);
    }

    rt_obstacles_.writeFromNonRT(snapshot);
}

void PotentialFieldController::publish_markers() {
    visualization_msgs::msg::MarkerArray msg;

    const auto& obstacles = *rt_obstacles_.readFromRT();
    int i = 0;
    for (const auto& obs : obstacles){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base";
        marker.header.stamp = get_node()->get_clock()->now();
        marker.ns = "";
        marker.id = i++;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = obs.position;
        marker.pose.orientation = obs.orientation;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        if (obs.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = 2 * obs.radius;
            marker.scale.y = 2 * obs.radius;
            marker.scale.z = 2 * obs.radius;
        }

        else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = obs.box_size.at(0);
            marker.scale.y = obs.box_size.at(1);
            marker.scale.z = obs.box_size.at(2);
        }

        // Push back to marker array
        msg.markers.push_back(marker);
    }

    pub_markers_->publish(msg);
}

// Debug print functions to reduce clutter in controller code
void PotentialFieldController::kdl_chain_error_print() {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree: ");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_link_ << " --> " << tip_link_);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfJoints() << " joints");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfSegments() << " segments");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  The segments are:");

    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for (it = segment_map.begin(); it != segment_map.end(); it++) {
        RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
}

void PotentialFieldController::kdl_chain_success_print() {
    RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");

    // debug: print kdl tree and kdl chain
    RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_link_.c_str(), tip_link_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d joints", tree_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d segments", tree_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_tree_ segments are:");

    // Print the segments of the KDL tree
    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;
    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
    RCLCPP_INFO(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", chain_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", chain_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_chain_ segments are:");
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
        const KDL::Segment& segment = chain_.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
    }
}

}  // namespace ex6
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ex6::PotentialFieldController,
                       controller_interface::ControllerInterface)