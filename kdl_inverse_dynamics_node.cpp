#include <memory>
#include <string>

#include "kdl/chain.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_msgs/msg/lbr_state.hpp"

class InverseDynamicsNode : public rclcpp::Node {
public:
  InverseDynamicsNode(const std::string &node_name) : rclcpp::Node(node_name) {
    lbr_state_sub_ = this->create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr_state", rclcpp::SystemDefaultsQoS(),
        std::bind(&InverseDynamicsNode::lbr_state_cb_, this, std::placeholders::_1));

    robot_state_publisher_param_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(
        "/robot_state_publisher/get_parameters", rmw_qos_profile_parameters);
    RCLCPP_INFO(this->get_logger(), "Waiting for /robot_state_publisher/get_parameters service...");
    robot_state_publisher_param_client_->wait_for_service();
    RCLCPP_INFO(this->get_logger(), "Done.");

    RCLCPP_INFO(this->get_logger(), "Retrieving /robot_description...");
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");
    auto future = robot_state_publisher_param_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    auto response = future.get()->values;
    robot_description_ = response[0].string_value;
    RCLCPP_INFO(this->get_logger(), "Done.");

    RCLCPP_INFO(this->get_logger(), "Creating KDL from robot_description...");
    kdl_parser::treeFromString(robot_description_, kdl_tree_);
    RCLCPP_INFO(this->get_logger(), "Done");

    RCLCPP_INFO(this->get_logger(), "Creating chain from tree...");
    kdl_tree_.getChain("lbr_link_0", "lbr_link_ee", kdl_chain_);
    RCLCPP_INFO(this->get_logger(), "Done.");

    RCLCPP_INFO(this->get_logger(), "Creating chain id solver...");
    kdl_chain_id_solver_ =
        std::make_shared<KDL::ChainIdSolver_RNE>(kdl_chain_, KDL::Vector(0., 0., 9.81));
    RCLCPP_INFO(this->get_logger(), "Done.");

    q_.data.resize(kdl_chain_.getNrOfJoints());
    dq_.data.resize(kdl_chain_.getNrOfJoints());
    ddq_.data.resize(kdl_chain_.getNrOfJoints());
    tau_.data.resize(kdl_chain_.getNrOfJoints());

    f_ext_ = std::vector<KDL::Wrench>(kdl_chain_.getNrOfSegments(),
                                      {KDL::Vector::Zero(), KDL::Vector::Zero()});
  }

protected:
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr robot_state_publisher_param_client_;

  std::string robot_description_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainIdSolver_RNE> kdl_chain_id_solver_;

  KDL::JntArray q_, dq_, ddq_, tau_;
  KDL::Wrenches f_ext_;

  void lbr_state_cb_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    RCLCPP_INFO(this->get_logger(), "measured torque: %f, %f, %f, %f, %f, %f, %f",
                lbr_state->measured_torque[0], lbr_state->measured_torque[1],
                lbr_state->measured_torque[2], lbr_state->measured_torque[3],
                lbr_state->measured_torque[4], lbr_state->measured_torque[5],
                lbr_state->measured_torque[6]);

    q_.data = Eigen::VectorXd::Map(lbr_state->measured_joint_position.data(),
                                   lbr_state->measured_joint_position.size());
    dq_.data.setZero();
    ddq_.data.setZero();

    kdl_chain_id_solver_->CartToJnt(q_, dq_, ddq_, f_ext_, tau_);

    RCLCPP_INFO(this->get_logger(), "calucated torque: %f, %f, %f, %f, %f, %f, %f", tau_.data[0],
                tau_.data[1], tau_.data[2], tau_.data[3], tau_.data[4], tau_.data[5], tau_.data[6]);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseDynamicsNode>("kdl_inverse_dynamics_node"));
  return 0;
}
