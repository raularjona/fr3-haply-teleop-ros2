#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class VirtualWallNode : public rclcpp::Node
{
public:

  VirtualWallNode() : Node("virtual_wall_node")
  {
    marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(
        "/virtual_wall_marker",10);

    timer_ =
      create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&VirtualWallNode::publishWall,this));
  }

private:

  void publishWall()
  {
    visualization_msgs::msg::Marker wall;

    wall.header.frame_id = "fr3_link0";
    wall.header.stamp = now();

    wall.ns = "virtual_wall";
    wall.id = 0;

    wall.type = visualization_msgs::msg::Marker::CUBE;
    wall.action = visualization_msgs::msg::Marker::ADD;

    wall.pose.position.x = 0.6;
    wall.pose.position.y = 0.0;
    wall.pose.position.z = 0.4;

    wall.pose.orientation.w = 1.0;

    wall.scale.x = 0.02;
    wall.scale.y = 1.0;
    wall.scale.z = 1.0;

    wall.color.r = 1.0;
    wall.color.g = 0.0;
    wall.color.b = 0.0;
    wall.color.a = 0.8;

    marker_pub_->publish(wall);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<VirtualWallNode>());
  rclcpp::shutdown();
  return 0;
}
