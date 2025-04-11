#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>

#include "kinematics.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"


using std::placeholders::_1;
using Eigen::Vector3d;
using Eigen::Vector4d;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MovementController : public rclcpp::Node
{
  public:
    MovementController() :
    Node("movement_controller"), count_(0)
    {
      flsP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/steering/front_left_steering", 10);
      frsP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/steering/front_right_steering", 10);
      rlsP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/steering/rear_left_steering", 10);
      rrsP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/steering/rear_right_steering", 10);

      fllwP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/wheel/front_left_left_wheel", 10);
      flrwP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/wheel/front_left_right_wheel", 10);
      frlwP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/wheel/front_right_left_wheel", 10);
      frrwP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/wheel/front_right_right_wheel", 10);
      rllwP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/wheel/rear_left_left_wheel", 10);
      rlrwP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/wheel/rear_left_right_wheel", 10);
      rrlwP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/wheel/rear_right_left_wheel", 10);
      rrrwP = this->create_publisher<std_msgs::msg::Float64>("/agro_bot/cmd_vel/wheel/rear_right_right_wheel", 10);

      cmdSub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&MovementController::cmd_callback, this, _1));
      jointStatesSub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&MovementController::joints_callback, this, _1));

      timer_ = this->create_wall_timer(
      50ms, std::bind(&MovementController::timer_callback, this));

    }

  private:
    Vector3d robotSpeedVector{0, 0, 0};
    Vector4d robotCurrentSteeringVector{0, 0, 0, 0};
    Vector4d robotCurrentSteeringSpeedVector{0, 0, 0, 0};
    Vector3d robotParams{ 1.57, 2.1, 0.237 };

    std_msgs::msg::Float64 fls;
    std_msgs::msg::Float64 frs;
    std_msgs::msg::Float64 rls;
    std_msgs::msg::Float64 rrs;

    std_msgs::msg::Float64 fllw;
    std_msgs::msg::Float64 flrw;
    std_msgs::msg::Float64 frlw;
    std_msgs::msg::Float64 frrw;
    std_msgs::msg::Float64 rllw;
    std_msgs::msg::Float64 rlrw;
    std_msgs::msg::Float64 rrlw;
    std_msgs::msg::Float64 rrrw;

    struct kinematicOut test;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr flsP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr frsP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rlsP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rrsP;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fllwP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr flrwP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr frlwP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr frrwP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rllwP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rlrwP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rrlwP;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rrrwP;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdSub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStatesSub;

    void timer_callback()
    {
        this->test = inversKinematic(this->robotSpeedVector,
                                     this->robotParams,
                                     this->robotCurrentSteeringVector,
                                     this->robotCurrentSteeringSpeedVector);

        fls.data = test.targSteerAngle(0);
        frs.data = test.targSteerAngle(3);
        rls.data = test.targSteerAngle(1);
        rrs.data = test.targSteerAngle(2);

        fllw.data = test.wheelsSpeed(0);
        flrw.data = test.wheelsSpeed(1);
        frlw.data = test.wheelsSpeed(6);
        frrw.data = test.wheelsSpeed(7);
        rllw.data = test.wheelsSpeed(2);
        rlrw.data = test.wheelsSpeed(3);
        rrlw.data = test.wheelsSpeed(4);
        rrrw.data = test.wheelsSpeed(5);

        flsP->publish(fls);
        frsP->publish(frs);
        rlsP->publish(rls);
        rrsP->publish(rrs);

        fllwP->publish(fllw);
        flrwP->publish(flrw);
        frlwP->publish(frlw);
        frrwP->publish(frrw);
        rllwP->publish(rllw);
        rlrwP->publish(rlrw);
        rrlwP->publish(rrlw);
        rrrwP->publish(rrrw);
    }

    void cmd_callback(const geometry_msgs::msg::Twist & msg)
    {
        this->robotSpeedVector(0) = (double)msg.linear.x;
        this->robotSpeedVector(1) = (double)msg.linear.y;
        this->robotSpeedVector(2) = (double)msg.angular.z;

    }
    void joints_callback(const sensor_msgs::msg::JointState & msg)
    {
        this->robotCurrentSteeringVector(0) = msg.position[1];
        this->robotCurrentSteeringVector(3) = msg.position[4];
        this->robotCurrentSteeringVector(1) = msg.position[7];
        this->robotCurrentSteeringVector(2) = msg.position[10];

        this->robotCurrentSteeringSpeedVector(0) = msg.velocity[1];
        this->robotCurrentSteeringSpeedVector(3) = msg.velocity[4];
        this->robotCurrentSteeringSpeedVector(1) = msg.velocity[7];
        this->robotCurrentSteeringSpeedVector(2) = msg.velocity[10];
    }

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovementController>());
  rclcpp::shutdown();
  return 0;
}