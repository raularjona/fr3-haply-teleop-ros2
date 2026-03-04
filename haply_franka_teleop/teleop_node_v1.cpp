#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/duration.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <array>
#include <cmath>

class HaplyFrankaTeleop : public rclcpp::Node {
public:
  HaplyFrankaTeleop(const std::string& robot_ip)
  : Node("franka_haply_velocity_controller"),
    robot_(robot_ip),
    running_(true)
  {
    RCLCPP_INFO(this->get_logger(),
                "Teleoperación con limitación de aceleración");

    robot_.setCollisionBehavior(
      {{20,20,20,20,20,20,20}},
      {{20,20,20,20,20,20,20}},
      {{20,20,20,20,20,20}},
      {{20,20,20,20,20,20}}
    );

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/haply_pose", 10,
      std::bind(&HaplyFrankaTeleop::haplyCallback,
                this, std::placeholders::_1));

    control_thread_ = std::thread(&HaplyFrankaTeleop::controlLoop, this);
  }

  ~HaplyFrankaTeleop() {
    running_ = false;
    if (control_thread_.joinable())
      control_thread_.join();
  }

private:

  void haplyCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (received_) {
      double dt =
        (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) -
        (last_stamp_.sec + last_stamp_.nanosec * 1e-9);

      if (dt > 0.0001) {
        vx_target_ =
          (msg->pose.position.x - last_pose_.pose.position.x) / dt;
        vy_target_ =
          (msg->pose.position.y - last_pose_.pose.position.y) / dt;
        vz_target_ =
          (msg->pose.position.z - last_pose_.pose.position.z) / dt;
      }
    }

    last_pose_ = *msg;
    last_stamp_ = msg->header.stamp;
    received_ = true;
  }

  void controlLoop() {

    while (!received_ && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!rclcpp::ok())
      return;

    try {

      robot_.control(
        [&](const franka::RobotState&,
            franka::Duration period)
        -> franka::CartesianVelocities {

          if (!rclcpp::ok() || !running_) {
            return franka::MotionFinished(
              franka::CartesianVelocities({0,0,0,0,0,0}));
          }

          double dt = period.toSec();
          if (dt <= 0) dt = 0.001;

          std::lock_guard<std::mutex> lock(mutex_);

          double max_acc = 1.0;   // m/s²
          double max_vel = 0.3;   // m/s

          auto ramp = [&](double v_cmd,
                          double v_target) {

            double diff = v_target - v_cmd;
            double max_step = max_acc * dt;

            if (std::abs(diff) > max_step)
              v_cmd += (diff > 0 ? max_step : -max_step);
            else
              v_cmd = v_target;

            return std::clamp(v_cmd,
                              -max_vel,
                               max_vel);
          };

          vx_cmd_ = ramp(vx_cmd_, vx_target_);
          vy_cmd_ = ramp(vy_cmd_, vy_target_);
          vz_cmd_ = ramp(vz_cmd_, vz_target_);

          return franka::CartesianVelocities(
            {vx_cmd_, vy_cmd_, vz_cmd_, 0,0,0});
        });

    } catch (const franka::ControlException& e) {

      RCLCPP_ERROR(this->get_logger(),
                   "ControlException: %s", e.what());
    }
  }

  franka::Robot robot_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

  geometry_msgs::msg::PoseStamped last_pose_;
  builtin_interfaces::msg::Time last_stamp_;

  double vx_target_{0}, vy_target_{0}, vz_target_{0};
  double vx_cmd_{0}, vy_cmd_{0}, vz_cmd_{0};

  std::mutex mutex_;
  std::thread control_thread_;
  std::atomic<bool> running_;
  bool received_{false};
};

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  if (argc != 2) {
    std::cerr << "Uso: ros2 run haply_franka_teleop teleop_node <robot_ip>"
              << std::endl;
    return -1;
  }

  auto node =
    std::make_shared<HaplyFrankaTeleop>(argv[1]);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}