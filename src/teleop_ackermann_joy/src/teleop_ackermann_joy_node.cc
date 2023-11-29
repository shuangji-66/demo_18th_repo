#include <string>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/Joy.h>
#include <ucar_common_msgs/ExtraCtrl.h>

using std::string;

class TeleopAckermannJoy
{
public:
    //! Constructor
    TeleopAckermannJoy();

    //! Run the node.
    void run();

private:
    // Ros infrastructure
    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber sub_joy_;
    ros::Publisher pub_acker_, pub_extra_ctrl_;
    ros::Time last_time_stamp_;
    int freq_, axis_throttle_, axis_steering_, axis_manual_break_, button_enable_, button_reverse_, button_max_break_, button_horn_, button_headlight_;
    bool is_reverse_, is_enable_, is_max_break_, is_horn_, is_headlight_;
    // workaround for joystick bug: inital value not actual the neutual value for the joystick, which result in the car moving when the joystick is not touched
    bool is_throttle_zeroed_;
    double max_speed_, max_steering_angle_, max_speed_backward_, max_break_force_, throttle_deadzone_, steering_center_deadzone_, man_break_deadzone_, throttle_scale_, steering_scale_, man_break_scale_, throttle_offset_, steering_offset_, man_break_offset_;
    double speed_setpoint, steering_setpoint, break_force_setpoint;

    void recivecJoystickInput(sensor_msgs::Joy input);
    void publishAckermannCtrl();
    void publishExtraCtrl();
};

TeleopAckermannJoy::TeleopAckermannJoy() : nh_private_("~")
{
    // Get parameters from the parameter server
    nh_private_.param<int>("rate", freq_, 100);
    nh_private_.param<int>("axis_throttle", axis_throttle_, 5);
    nh_private_.param<int>("axis_steering", axis_steering_, 0);
    nh_private_.param<int>("axis_manual_break", axis_manual_break_, 3);
    nh_private_.param<int>("button_enable", button_enable_, 4);
    nh_private_.param<int>("button_reverse", button_reverse_, 1);
    nh_private_.param<int>("button_max_break", button_max_break_, 5);
    nh_private_.param<int>("button_horn", button_horn_, 0);
    nh_private_.param<int>("button_headlight", button_headlight_, 2);
    nh_private_.param<double>("max_speed", max_speed_, 0.2);
    nh_private_.param<double>("max_steering_angle", max_steering_angle_, 0.7);
    nh_private_.param<double>("max_speed_reverse", max_speed_backward_, -0.5);
    nh_private_.param<double>("max_break_force", max_break_force_, 0.0);
    nh_private_.param<double>("throttle_scale", throttle_scale_, 1.0);
    nh_private_.param<double>("throttle_deadzone", throttle_deadzone_, 0.02);
    nh_private_.param<double>("throttle_offset", throttle_offset_, 0.0);
    nh_private_.param<double>("steering_scale", steering_scale_, 1.0);
    nh_private_.param<double>("steering_center_deadzone", steering_center_deadzone_, 0.02);
    nh_private_.param<double>("steering_offset", steering_offset_, 0.0);
    nh_private_.param<double>("man_break_scale", man_break_scale_, 1.0);
    nh_private_.param<double>("man_break_deadzone", man_break_deadzone_, 0.02);
    nh_private_.param<double>("man_break_offset", man_break_offset_, 0.0);

    // publisher and subscriber
    pub_acker_ = nh_.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 1);
    pub_extra_ctrl_ = nh_.advertise<ucar_common_msgs::ExtraCtrl>("extra_ctrl", 1);
    sub_joy_ = nh_.subscribe("joy", 1, &TeleopAckermannJoy::recivecJoystickInput, this);
}

void TeleopAckermannJoy::recivecJoystickInput(sensor_msgs::Joy input)
{
    // process enable input
    if (input.buttons[button_enable_] == 1)
    {
        is_enable_ = true;
    }
    else
    {
        is_enable_ = false;
    }

    // process reverse input
    if (input.buttons[button_reverse_] == 1)
    {
        is_reverse_ = true;
    }
    else
    {
        is_reverse_ = false;
    }

    // process max break input
    if (input.buttons[button_max_break_] == 1)
    {
        is_max_break_ = true;
    }
    else
    {
        is_max_break_ = false;
    }

    // process horn input
    if (input.buttons[button_horn_] == 1)
    {
        is_horn_ = true;
    }
    else
    {
        is_horn_ = false;
    }

    // process headlight input
    if (input.buttons[button_headlight_] == 1)
    {
        is_headlight_ = true;
    }
    else
    {
        is_headlight_ = false;
    }

    // process steering input
    double steering_input = input.axes[axis_steering_];
    steering_input += steering_offset_;
    steering_input *= steering_scale_;
    if (std::abs(steering_input) < steering_center_deadzone_)
    {
        steering_setpoint = 0.0;
    }
    else
    {
        steering_setpoint = steering_input * max_steering_angle_;
    }

    // process break input
    double break_input = input.axes[axis_manual_break_];
    break_input += man_break_offset_;
    break_input *= man_break_scale_;
    if (std::abs(break_input) < man_break_deadzone_)
    {
        break_force_setpoint = 0.0;
    }
    else
    {
        break_force_setpoint = break_input * max_break_force_;
    }

    // process throttle input
    double throttle_input = input.axes[axis_throttle_];
    throttle_input += throttle_offset_;
    throttle_input *= throttle_scale_;
    if (!is_throttle_zeroed_)
    {
        if (throttle_input - throttle_deadzone_ < 10e-6)
        {
            is_throttle_zeroed_ = true;
            ROS_INFO("Throttle Armed!!!!!!");
        }
        else
        {
            ROS_WARN("Throttle not zeroed, please release the throttle to zero position");
            speed_setpoint = 0.0;
            return;
        }
    }

    if (throttle_input > throttle_deadzone_)
    {
        // throttle_input = (input.axes[axis_throttle_] - throttle_deadzone_) / (1 - throttle_deadzone_);
        if (is_reverse_)
        {
            speed_setpoint = throttle_input * max_speed_backward_;
        }
        else
        {
            speed_setpoint = throttle_input * max_speed_;
        }
    }
    else
    {

        speed_setpoint = 0.0;
    }
}

void TeleopAckermannJoy::publishAckermannCtrl()
{
    ackermann_msgs::AckermannDrive acker_ctrl;
    acker_ctrl.steering_angle = steering_setpoint;
    acker_ctrl.speed = speed_setpoint;
    acker_ctrl.acceleration = 0.0;
    acker_ctrl.jerk = 0.0;
    acker_ctrl.steering_angle_velocity = 0.0;

    if (!is_enable_)
    {
        acker_ctrl.speed = 0.0;
        acker_ctrl.steering_angle = 0.0;
    }

    pub_acker_.publish(acker_ctrl);
}

void TeleopAckermannJoy::publishExtraCtrl()
{
    ucar_common_msgs::ExtraCtrl extra_ctrl;
    extra_ctrl.mechanical_break = is_max_break_ ? 1.0 : break_force_setpoint;
    extra_ctrl.horn = is_horn_;
    extra_ctrl.headlight = is_headlight_;
    pub_extra_ctrl_.publish(extra_ctrl);
}

void TeleopAckermannJoy::run()
{

    while (ros::ok())
    {
        ros::spinOnce();
        publishAckermannCtrl();
        publishExtraCtrl();
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / freq_));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_ackermann_joy");

    TeleopAckermannJoy node;
    node.run();

    return 0;
}
