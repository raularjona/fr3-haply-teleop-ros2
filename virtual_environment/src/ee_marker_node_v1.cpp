#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class EEMarkerNode : public rclcpp::Node
{
public:

  EEMarkerNode() : Node("ee_marker_node")
  {
    marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(
        "/ee_marker",10);

    sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/franka_ee_pose",
        10,
        std::bind(&EEMarkerNode::poseCallback,this,std::placeholders::_1));

    RCLCPP_INFO(get_logger(),"EE marker node started");
  }

private:

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "fr3_link0";
    marker.header.stamp = now();

    marker.ns = "ee";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = msg->pose;

    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<EEMarkerNode>());
  rclcpp::shutdown();
  return 0;
}