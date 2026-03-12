#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <algorithm>

class HapticWallNode : public rclcpp::Node
{
public:

  HapticWallNode() : Node("franka_haptic_wall")
  {
    sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/franka_ee_pose",
        10,
        std::bind(&HapticWallNode::poseCallback,this,std::placeholders::_1));

    force_pub_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/haply_force",10);

    RCLCPP_INFO(get_logger(),"Haptic wall node started");
  }

private:

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double x = msg->pose.position.x;

    double penetration = x - wall_x_;
    penetration = std::min(penetration, 0.02);

    double force = 0.0;

    if(penetration > 0)
    {
      force = stiffness_ * penetration;
    }
    force = std::clamp(force,-4.0,4.0);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      200,
      "penetration = %.4f  force = %.2f",
      penetration,
      force
    );

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "EE x = %.3f",
      x
    );
    geometry_msgs::msg::WrenchStamped wrench;

    wrench.header.stamp = now();

    wrench.wrench.force.x = force;
    wrench.wrench.force.y = 0.0;
    wrench.wrench.force.z = 0.0;

    force_pub_->publish(wrench);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;

  double wall_x_ = 0.6;
  double stiffness_ = 400.0;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<HapticWallNode>());
  rclcpp::shutdown();
  return 0;
}