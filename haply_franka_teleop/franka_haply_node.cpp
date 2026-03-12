#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <franka/robot.h>
#include <franka/exception.h>

class FrankaHaplyNode : public rclcpp::Node {
public:

  enum class ControlState { kMoveToInit, kTeleop };

  FrankaHaplyNode() : Node("franka_haply_velocity_controller") {

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/haply_pose", 10,
      std::bind(&FrankaHaplyNode::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Nodo Franka-Haply VELOCITY iniciado. Esperando datos...");
  }

  void run_robot(const std::string& robot_ip) {

    try {

      franka::Robot robot(robot_ip);

      robot.setCollisionBehavior(
        {{20,20,18,18,16,14,12}}, {{20,20,18,18,16,14,12}},
        {{20,20,18,18,16,14,12}}, {{20,20,18,18,16,14,12}},
        {{20,20,20,10,10,10}}, {{20,20,20,10,10,10}},
        {{20,20,20,10,10,10}}, {{20,20,20,10,10,10}});

      ControlState current_state = ControlState::kMoveToInit;

      geometry_msgs::msg::Pose haply_initial_pose;
      bool haply_initialized = false;

      const double target_x = 0.40;
      const double target_y = 0.00;
      const double target_z = 0.50;

      const double max_vel = 0.20;   // m/s
      const double max_acc = 0.5;    // m/s² (MUY IMPORTANTE)

      std::array<double,3> velocity = {0.0, 0.0, 0.0};

      RCLCPP_INFO(this->get_logger(),
        "Iniciando control en velocidad (1kHz)...");

      robot.control(
        [&](const franka::RobotState& state,
            franka::Duration period) -> franka::CartesianVelocities {

        std::lock_guard<std::mutex> lock(pose_mutex_);

        double dt = period.toSec();

        double current_x = state.O_T_EE[12];
        double current_y = state.O_T_EE[13];
        double current_z = state.O_T_EE[14];

        double desired_vx = 0.0;
        double desired_vy = 0.0;
        double desired_vz = 0.0;

        // =====================================================
        // ESTADO 1: MOVER A POSICIÓN NEUTRA
        // =====================================================
        if (current_state == ControlState::kMoveToInit) {

          double dx = target_x - current_x;
          double dy = target_y - current_y;
          double dz = target_z - current_z;

          double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

          if (distance < 0.01) {
            current_state = ControlState::kTeleop;
            haply_initial_pose = latest_pose_;
            haply_initialized = true;

            RCLCPP_INFO(this->get_logger(),
              "Posicion neutra alcanzada. TELEOPERACION ACTIVA.");
          } else {
            desired_vx = std::clamp(dx, -max_vel, max_vel);
            desired_vy = std::clamp(dy, -max_vel, max_vel);
            desired_vz = std::clamp(dz, -max_vel, max_vel);
          }
        }

        // =====================================================
        // ESTADO 2: TELEOPERACIÓN
        // =====================================================
        else if (current_state == ControlState::kTeleop && haply_initialized) {

          double delta_x =
            latest_pose_.position.x - haply_initial_pose.position.x;
          double delta_y =
            latest_pose_.position.y - haply_initial_pose.position.y;
          double delta_z =
            latest_pose_.position.z - haply_initial_pose.position.z;

          double scale = 0.8;

          desired_vx =
            std::clamp(scale * delta_x, -max_vel, max_vel);
          desired_vy =
            std::clamp(scale * delta_y, -max_vel, max_vel);
          desired_vz =
            std::clamp(scale * delta_z, -max_vel, max_vel);
        }

        // =====================================================
        // LIMITADOR DE ACELERACIÓN (ANTI-REFLEX)
        // =====================================================
        for (int i = 0; i < 3; i++) {

          double desired =
            (i == 0 ? desired_vx :
             i == 1 ? desired_vy :
                      desired_vz);

          double delta_v = desired - velocity[i];

          double max_delta = max_acc * dt;

          delta_v = std::clamp(delta_v, -max_delta, max_delta);

          velocity[i] += delta_v;
        }

        franka::CartesianVelocities output =
          {{velocity[0], velocity[1], velocity[2], 0.0, 0.0, 0.0}};

        return output;
      });

    } catch (const franka::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),
        "FRANKA ERROR: %s", e.what());
    }
  }

private:

  void topic_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    latest_pose_ = msg->pose;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  geometry_msgs::msg::Pose latest_pose_;
  std::mutex pose_mutex_;
};

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Uso: ros2 run haply_franka_teleop teleop_node <IP_ROBOT>"
              << std::endl;
    return -1;
  }

  auto node = std::make_shared<FrankaHaplyNode>();

  std::thread ros_thread([&]() { rclcpp::spin(node); });

  node->run_robot(argv[1]);

  if (ros_thread.joinable())
    ros_thread.join();

  rclcpp::shutdown();
  return 0;
}