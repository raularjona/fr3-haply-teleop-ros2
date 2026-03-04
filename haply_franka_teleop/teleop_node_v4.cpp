#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/duration.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <array>
#include <cmath>
#include <algorithm>

class HaplyFrankaTeleop : public rclcpp::Node {
public:
  HaplyFrankaTeleop(const std::string& robot_ip)
  : Node("franka_haply_velocity_controller"),
    robot_(robot_ip),
    running_(true)
  {
    RCLCPP_INFO(this->get_logger(),
      "Teleoperación híbrida estable");

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

  // =============================
  // CALLBACK HAPLY
  // =============================
  void haplyCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    haply_current_ = msg->pose.position;

    if (!haply_initialized_) {
      haply_home_ = haply_current_;
      haply_initialized_ = true;
    }

    received_ = true;
  }

  // =============================
  // LOOP PRINCIPAL
  // =============================
  void controlLoop() {

    while (!received_ && rclcpp::ok())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (!rclcpp::ok())
      return;

    try {

      // ==========================================
      // HOME MIN-JERK
      // ==========================================
      std::array<double,7> home = {
        0.0, -0.5, 0.0, -2.2, 0.0, 1.7, 0.8
      };

      RCLCPP_INFO(this->get_logger(), "Moviendo a HOME...");

      std::array<double,7> q_start;
      bool initialized = false;
      double time = 0.0;
      double duration = 4.0;

      robot_.control(
        [&](const franka::RobotState& state,
            franka::Duration period)
        -> franka::JointPositions {

          if (!initialized) {
            q_start = state.q;
            initialized = true;
          }

          time += period.toSec();
          double r = std::min(time / duration, 1.0);

          double s =
            10*pow(r,3) - 15*pow(r,4) + 6*pow(r,5);

          std::array<double,7> q_d;
          for (size_t i = 0; i < 7; i++)
            q_d[i] = q_start[i] + s*(home[i] - q_start[i]);

          if (r >= 1.0)
            return franka::MotionFinished(
              franka::JointPositions(home));

          return franka::JointPositions(q_d);
        });

      RCLCPP_INFO(this->get_logger(), "HOME alcanzado.");

      // ==========================================
      // FASE DE TRANSICIÓN SUAVE
      // ==========================================
      double settle_time = 0.5;
      double t_settle = 0.0;

      robot_.control(
        [&](const franka::RobotState&,
            franka::Duration period)
        -> franka::CartesianVelocities {

          t_settle += period.toSec();

          if (t_settle >= settle_time)
            return franka::MotionFinished(
              franka::CartesianVelocities({0,0,0,0,0,0}));

          return franka::CartesianVelocities({0,0,0,0,0,0});
        });

      RCLCPP_INFO(this->get_logger(),
                  "Transición completada.");

      // ==========================================
      // SINCRONIZAR REFERENCIAS
      // ==========================================
      franka::RobotState state = robot_.readOnce();

      robot_home_x_ = state.O_T_EE[12];
      robot_home_y_ = state.O_T_EE[13];
      robot_home_z_ = state.O_T_EE[14];

      {
        std::lock_guard<std::mutex> lock(mutex_);
        haply_home_ = haply_current_;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Iniciando teleoperación estable.");

      // ==========================================
      // CONTROL PROPORCIONAL CARTESIANO
      // ==========================================
      robot_.control(
        [&](const franka::RobotState& state,
            franka::Duration)
        -> franka::CartesianVelocities {

          if (!rclcpp::ok() || !running_)
            return franka::MotionFinished(
              franka::CartesianVelocities({0,0,0,0,0,0}));

          double x_r = state.O_T_EE[12];
          double y_r = state.O_T_EE[13];
          double z_r = state.O_T_EE[14];

          double dx_h, dy_h, dz_h;

          {
            std::lock_guard<std::mutex> lock(mutex_);
            dx_h = haply_current_.x - haply_home_.x;
            dy_h = haply_current_.y - haply_home_.y;
            dz_h = haply_current_.z - haply_home_.z;
          }

          double scale = 1.0;

          double x_d = robot_home_x_ + scale*dx_h;
          double y_d = robot_home_y_ + scale*dy_h;
          double z_d = robot_home_z_ + scale*dz_h;

          double ex = x_d - x_r;
          double ey = y_d - y_r;
          double ez = z_d - z_r;

          double Kp = 3.0;

          double vx = Kp * ex;
          double vy = Kp * ey;
          double vz = Kp * ez;

          double max_vel = 0.3;

          vx = std::clamp(vx, -max_vel, max_vel);
          vy = std::clamp(vy, -max_vel, max_vel);
          vz = std::clamp(vz, -max_vel, max_vel);

          return franka::CartesianVelocities(
            {vx, vy, vz, 0.0, 0.0, 0.0});
        });

    } catch (const franka::ControlException& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "ControlException: %s", e.what());
    }
  }

  // =============================
  // VARIABLES
  // =============================
  franka::Robot robot_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

  geometry_msgs::msg::Point haply_home_;
  geometry_msgs::msg::Point haply_current_;
  bool haply_initialized_{false};

  double robot_home_x_{0};
  double robot_home_y_{0};
  double robot_home_z_{0};

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