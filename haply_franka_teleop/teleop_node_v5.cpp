#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/duration.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 
#include <builtin_interfaces/msg/time.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <array>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

class HaplyFrankaTeleop : public rclcpp::Node {
public:
  HaplyFrankaTeleop(const std::string& robot_ip)
  : Node("franka_haply_velocity_controller"),
    robot_(robot_ip),
    running_(true)
  {
    RCLCPP_INFO(this->get_logger(), "Teleoperación híbrida con re-mapeo de ejes y RViz");

    // Configuración de seguridad (umbral de 20Nm/20N)
    robot_.setCollisionBehavior(
      {{20,20,20,20,20,20,20}}, {{20,20,20,20,20,20,20}},
      {{20,20,20,20,20,20}}, {{20,20,20,20,20,20}}
    );

    // Publicador para visualización en tiempo real
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Suscripción al tópico del Haply
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/haply_pose", 10,
      std::bind(&HaplyFrankaTeleop::haplyCallback, this, std::placeholders::_1));

    control_thread_ = std::thread(&HaplyFrankaTeleop::controlLoop, this);
  }

  ~HaplyFrankaTeleop() {
    running_ = false;
    if (control_thread_.joinable())
      control_thread_.join();
  }

private:
  // Publica el estado a RViz solo cada ~33ms para evitar sobrecargar la CPU
  void publishToRviz(const franka::RobotState& state) {
    if (++rviz_counter_ >= 33) { 
      auto js_msg = sensor_msgs::msg::JointState();
      js_msg.header.stamp = this->now();
      js_msg.name = {"fr3_joint1", "fr3_joint2", "fr3_joint3", 
                     "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
      for (size_t i = 0; i < 7; ++i) {
        js_msg.position.push_back(state.q[i]);
      }
      joint_pub_->publish(js_msg);
      rviz_counter_ = 0;
    }
  }

  void haplyCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    haply_current_ = msg->pose.position;
    if (!haply_initialized_) {
      haply_home_ = haply_current_;
      haply_initialized_ = true;
    }
    received_ = true;
  }

  void controlLoop() {
    while (!received_ && rclcpp::ok())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (!rclcpp::ok()) return;

    try {
      // 1. MOVIMIENTO INICIAL A POSICIÓN HOME (Joint Control)
      std::array<double,7> home = {0.0, -0.8, 0.0, -2.6, 0.0, 1.7, 0.8};
      RCLCPP_INFO(this->get_logger(), "Moviendo a HOME...");

      std::array<double,7> q_start;
      bool initialized = false;
      double time = 0.0;
      double duration = 4.0;

      robot_.control(
        [&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
          if (!initialized) { q_start = state.q; initialized = true; }
          publishToRviz(state); 

          time += period.toSec();
          double r = std::min(time / duration, 1.0);
          double s = 10*pow(r,3) - 15*pow(r,4) + 6*pow(r,5); // Quinta de suavizado

          std::array<double,7> q_d;
          for (size_t i = 0; i < 7; i++)
            q_d[i] = q_start[i] + s*(home[i] - q_start[i]);

          if (r >= 1.0) return franka::MotionFinished(franka::JointPositions(home));
          return franka::JointPositions(q_d);
        });

      // 2. FASE DE ESTABILIZACIÓN
      double settle_time = 0.5;
      double t_settle = 0.0;
      robot_.control(
        [&](const franka::RobotState& state, franka::Duration period) -> franka::CartesianVelocities {
          t_settle += period.toSec();
          publishToRviz(state);
          if (t_settle >= settle_time)
            return franka::MotionFinished(franka::CartesianVelocities({0,0,0,0,0,0}));
          return franka::CartesianVelocities({0,0,0,0,0,0});
        });

      // 3. TELEOPERACIÓN ACTIVA CON CAMBIO DE EJES
      franka::RobotState initial_state = robot_.readOnce();
      robot_home_x_ = initial_state.O_T_EE[12];
      robot_home_y_ = initial_state.O_T_EE[13];
      robot_home_z_ = initial_state.O_T_EE[14];

      {
        std::lock_guard<std::mutex> lock(mutex_);
        haply_home_ = haply_current_; // Sincroniza punto cero relativo
      }

      RCLCPP_INFO(this->get_logger(), "Iniciando teleoperación. El Franka sigue al Haply.");

      robot_.control(
        [&](const franka::RobotState& state, franka::Duration) -> franka::CartesianVelocities {
          if (!rclcpp::ok() || !running_)
            return franka::MotionFinished(franka::CartesianVelocities({0,0,0,0,0,0}));

          publishToRviz(state); 

          double x_r = state.O_T_EE[12];
          double y_r = state.O_T_EE[13];
          double z_r = state.O_T_EE[14];

          double dx_h, dy_h, dz_h;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            // Deltas crudos del dispositivo maestro
            double raw_x = haply_current_.x - haply_home_.x;
            double raw_y = haply_current_.y - haply_home_.y;
            double raw_z = haply_current_.z - haply_home_.z;

            // CAMBIO DE EJES: Mapeo para que el frente del Haply sea el frente del Franka
            dx_h = -raw_y;  // Haply adelante/atrás -> Franka X
            dy_h = raw_x; // Haply izq/der -> Franka Y (Invertido para evitar espejo)
            dz_h = raw_z;  // Haply arriba/abajo -> Franka Z
          }

          double scale = 1.2; // Escala de movimiento
          double x_d = robot_home_x_ + scale * dx_h;
          double y_d = robot_home_y_ + scale * dy_h;
          double z_d = robot_home_z_ + scale * dz_h;

          // Ganancia Proporcional y limitación de seguridad
          double Kp = 3.5;
          double vx = std::clamp(Kp * (x_d - x_r), -0.3, 0.3);
          double vy = std::clamp(Kp * (y_d - y_r), -0.3, 0.3);
          double vz = std::clamp(Kp * (z_d - z_r), -0.3, 0.3);

          return franka::CartesianVelocities({vx, vy, vz, 0.0, 0.0, 0.0});
        });

    } catch (const franka::ControlException& e) {
      RCLCPP_ERROR(this->get_logger(), "ControlException: %s", e.what());
    } catch (const franka::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error de libfranka: %s", e.what());
    }
  }

  // Miembros de la clase
  franka::Robot robot_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

  geometry_msgs::msg::Point haply_home_;
  geometry_msgs::msg::Point haply_current_;
  bool haply_initialized_{false};

  double robot_home_x_{0}, robot_home_y_{0}, robot_home_z_{0};
  int rviz_counter_{0};

  std::mutex mutex_;
  std::thread control_thread_;
  std::atomic<bool> running_;
  bool received_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  if (argc != 2) {
    std::cerr << "Error. Uso: ros2 run haply_franka_teleop teleop_node <IP_ROBOT>" << std::endl;
    return -1;
  }
  auto node = std::make_shared<HaplyFrankaTeleop>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}