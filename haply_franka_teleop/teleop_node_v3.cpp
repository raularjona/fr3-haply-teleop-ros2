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
      "Teleoperación con HOME (min-jerk) + filtros dinámicos");

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

    if (received_) {

      double current_time =
        msg->header.stamp.sec +
        msg->header.stamp.nanosec * 1e-9;

      double last_time =
        last_stamp_.sec +
        last_stamp_.nanosec * 1e-9;

      double dt = current_time - last_time;

      if (dt > 0.0001) {

        vx_target_ =
          (msg->pose.position.x - last_pose_.pose.position.x) / dt;
        vy_target_ =
          (msg->pose.position.y - last_pose_.pose.position.y) / dt;
        vz_target_ =
          (msg->pose.position.z - last_pose_.pose.position.z) / dt;

        // Deadband
        if (std::abs(vx_target_) < 0.002) vx_target_ = 0;
        if (std::abs(vy_target_) < 0.002) vy_target_ = 0;
        if (std::abs(vz_target_) < 0.002) vz_target_ = 0;

        // Clamp entrada
        double max_input_vel = 0.5;
        vx_target_ = std::clamp(vx_target_, -max_input_vel, max_input_vel);
        vy_target_ = std::clamp(vy_target_, -max_input_vel, max_input_vel);
        vz_target_ = std::clamp(vz_target_, -max_input_vel, max_input_vel);
      }
    }

    last_pose_ = *msg;
    last_stamp_ = msg->header.stamp;
    received_ = true;
  }

  // =============================
  // LOOP PRINCIPAL
  // =============================
  void controlLoop() {

    while (!received_ && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!rclcpp::ok())
      return;

    try {

      // ==========================================
      // 1️⃣ MOVER A HOME CON PERFIL MIN-JERK
      // ==========================================
      std::array<double,7> home = {
        0.0,
       -0.5,
        0.0,
       -2.2,
        0.0,
        1.7,
        0.8
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
            10*std::pow(r,3)
          - 15*std::pow(r,4)
          +  6*std::pow(r,5);

          std::array<double,7> q_d;

          for (size_t i = 0; i < 7; i++) {
            q_d[i] =
              q_start[i] +
              s * (home[i] - q_start[i]);
          }

          if (r >= 1.0) {
            RCLCPP_INFO(this->get_logger(),
                        "HOME alcanzado.");
            return franka::MotionFinished(
              franka::JointPositions(home));
          }

          return franka::JointPositions(q_d);
        });

      RCLCPP_INFO(this->get_logger(),
                  "Iniciando teleoperación en velocidad...");

      // ==========================================
      // 2️⃣ CONTROL CARTESIANO CON FILTROS
      // ==========================================
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

          // ==============================
          // 1️⃣ FILTRO PASA BAJO
          // ==============================
          double tau = 0.07;
          double alpha = dt / (tau + dt);

          vx_filtered_ += alpha * (vx_target_ - vx_filtered_);
          vy_filtered_ += alpha * (vy_target_ - vy_filtered_);
          vz_filtered_ += alpha * (vz_target_ - vz_filtered_);

          // ==============================
          // 2️⃣ LIMITADOR DE ACELERACIÓN
          // ==============================
          double max_acc = 0.8;
          double max_vel = 0.25;

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

          vx_cmd_ = ramp(vx_cmd_, vx_filtered_);
          vy_cmd_ = ramp(vy_cmd_, vy_filtered_);
          vz_cmd_ = ramp(vz_cmd_, vz_filtered_);

          return franka::CartesianVelocities(
            {vx_cmd_, vy_cmd_, vz_cmd_, 0.0, 0.0, 0.0});
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

  geometry_msgs::msg::PoseStamped last_pose_;
  builtin_interfaces::msg::Time last_stamp_;

  double vx_target_{0}, vy_target_{0}, vz_target_{0};
  double vx_filtered_{0}, vy_filtered_{0}, vz_filtered_{0};
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