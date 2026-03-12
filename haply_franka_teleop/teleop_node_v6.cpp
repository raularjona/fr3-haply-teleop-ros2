#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/duration.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
      "Teleoperación con medición de fuerzas externas");

    // límites de colisión más tolerantes
    robot_.setCollisionBehavior(
      {{40,40,40,40,40,40,40}},
      {{40,40,40,40,40,40,40}},
      {{40,40,40,40,40,40}},
      {{40,40,40,40,40,40}}
    );

    joint_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states",10);
    // Debajo de joint_pub_
    
    ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/franka_ee_pose", 10);
    sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/haply_pose",
        10,
        std::bind(
          &HaplyFrankaTeleop::haplyCallback,
          this,
          std::placeholders::_1));

    control_thread_ =
      std::thread(&HaplyFrankaTeleop::controlLoop,this);
  }

  ~HaplyFrankaTeleop(){

    running_ = false;

    if(control_thread_.joinable())
      control_thread_.join();
  }

private:

  void publishToRviz(const franka::RobotState& state){

    if(++rviz_counter_ >= 33){

      sensor_msgs::msg::JointState js_msg;

      js_msg.header.stamp = this->now();

      js_msg.name = {
        "fr3_joint1","fr3_joint2","fr3_joint3",
        "fr3_joint4","fr3_joint5","fr3_joint6","fr3_joint7"
      };

      for(size_t i=0;i<7;i++)
        js_msg.position.push_back(state.q[i]);

      joint_pub_->publish(js_msg);

      rviz_counter_ = 0;
    }
  }

  void haplyCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg){

    std::lock_guard<std::mutex> lock(mutex_);

    haply_current_ = msg->pose.position;

    if(!haply_initialized_){

      haply_home_ = haply_current_;
      haply_initialized_ = true;
    }

    received_ = true;
  }

  void controlLoop(){

    while(!received_ && rclcpp::ok())
      std::this_thread::sleep_for(
        std::chrono::milliseconds(1));

    if(!rclcpp::ok()) return;

    try{

      std::array<double,7> home =
        {0.0,-0.8,0.0,-2.6,0.0,1.7,0.8};

      RCLCPP_INFO(
        this->get_logger(),
        "Moviendo robot a HOME");

      std::array<double,7> q_start;

      bool initialized=false;

      double time=0;
      double duration=4;

      robot_.control(
        [&](const franka::RobotState& state,
        franka::Duration period)
        -> franka::JointPositions{

        if(!initialized){

          q_start = state.q;
          initialized=true;
        }

        publishToRviz(state);

        time += period.toSec();

        double r = std::min(time/duration,1.0);

        double s =
          10*pow(r,3)
          -15*pow(r,4)
          +6*pow(r,5);

        std::array<double,7> q_d;

        for(size_t i=0;i<7;i++)
          q_d[i] =
            q_start[i] +
            s*(home[i]-q_start[i]);

        if(r>=1.0)
          return franka::MotionFinished(
            franka::JointPositions(home));

        return franka::JointPositions(q_d);
      });

      franka::RobotState initial_state =
        robot_.readOnce();

      robot_home_x_ = initial_state.O_T_EE[12];
      robot_home_y_ = initial_state.O_T_EE[13];
      robot_home_z_ = initial_state.O_T_EE[14];

      {
        std::lock_guard<std::mutex> lock(mutex_);
        haply_home_ = haply_current_;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Teleoperación activa");

      robot_.control(
        [&](const franka::RobotState& state,
        franka::Duration)
        -> franka::CartesianVelocities{

        if(!running_ || !rclcpp::ok())
          return franka::MotionFinished(
            franka::CartesianVelocities(
              {0,0,0,0,0,0}));

        publishToRviz(state);

        // ---------------------------
        // MEDICIÓN DE FUERZA
        // ---------------------------

        double fx = state.O_F_ext_hat_K[0];
        double fy = state.O_F_ext_hat_K[1];
        double fz = state.O_F_ext_hat_K[2];

        if(!bias_initialized_){

          bias_fx_ = fx;
          bias_fy_ = fy;
          bias_fz_ = fz;

          bias_initialized_ = true;

          RCLCPP_INFO(
            this->get_logger(),
            "Bias de fuerza calibrado");
        }

        fx -= bias_fx_;
        fy -= bias_fy_;
        fz -= bias_fz_;

        // filtro paso bajo

        double alpha = 0.1;

        filtered_fx_ =
          (1-alpha)*filtered_fx_ + alpha*fx;

        filtered_fy_ =
          (1-alpha)*filtered_fy_ + alpha*fy;

        filtered_fz_ =
          (1-alpha)*filtered_fz_ + alpha*fz;

        // velocidad EE

        double vx = state.O_dP_EE_c[0];
        double vy = state.O_dP_EE_c[1];
        double vz = state.O_dP_EE_c[2];

        double vel_norm =
          sqrt(vx*vx + vy*vy + vz*vz);

        double force_norm =
          sqrt(filtered_fx_*filtered_fx_ +
               filtered_fy_*filtered_fy_ +
               filtered_fz_*filtered_fz_);

        bool contact = false;

        if(force_norm > 6.0 && vel_norm < 0.05)
          contact = true;

        if(contact){

          RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            200,
            "CONTACTO DETECTADO -> F=%.2f",
            force_norm);
        }

        // ---------------------------
        // TELEOPERACIÓN
        // ---------------------------

        double x_r = state.O_T_EE[12];
        double y_r = state.O_T_EE[13];
        double z_r = state.O_T_EE[14];

        double dx_h,dy_h,dz_h;

        {
          std::lock_guard<std::mutex> lock(mutex_);

          double raw_x =
            haply_current_.x - haply_home_.x;

          double raw_y =
            haply_current_.y - haply_home_.y;

          double raw_z =
            haply_current_.z - haply_home_.z;

          dx_h = -raw_x;
          dy_h = -raw_y;
          dz_h = raw_z;
        }

        double scale = 1.2;

        double x_d = robot_home_x_ + scale*dx_h;
        double y_d = robot_home_y_ + scale*dy_h;
        double z_d = robot_home_z_ + scale*dz_h;

        double Kp = 3.5;

        double vx_cmd =
          std::clamp(Kp*(x_d-x_r),-0.15,0.15);

        double vy_cmd =
          std::clamp(Kp*(y_d-y_r),-0.15,0.15);

        double vz_cmd =
          std::clamp(Kp*(z_d-z_r),-0.15,0.15);

        return franka::CartesianVelocities(
          {vx_cmd,vy_cmd,vz_cmd,0,0,0});
      });

    }

    catch(const franka::ControlException& e){

      RCLCPP_ERROR(
        this->get_logger(),
        "ControlException: %s",
        e.what());
    }

    catch(const franka::Exception& e){

      RCLCPP_ERROR(
        this->get_logger(),
        "Franka Exception: %s",
        e.what());
    }
  }

  franka::Robot robot_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  
  geometry_msgs::msg::Point haply_home_;
  geometry_msgs::msg::Point haply_current_;

  bool haply_initialized_{false};

  double robot_home_x_{0};
  double robot_home_y_{0};
  double robot_home_z_{0};

  int rviz_counter_{0};

  std::mutex mutex_;

  std::thread control_thread_;

  std::atomic<bool> running_;

  bool received_{false};

  double bias_fx_{0};
  double bias_fy_{0};
  double bias_fz_{0};

  bool bias_initialized_{false};

  double filtered_fx_{0};
  double filtered_fy_{0};
  double filtered_fz_{0};
};

int main(int argc,char** argv){

  rclcpp::init(argc,argv);

  if(argc!=2){

    std::cerr
      <<"Uso: ros2 run haply_franka_teleop teleop_node <robot_ip>"
      <<std::endl;

    return -1;
  }

  auto node =
    std::make_shared<HaplyFrankaTeleop>(argv[1]);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}