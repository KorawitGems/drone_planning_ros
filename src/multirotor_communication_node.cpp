// MIT License

// Copyright (c) 2021 Kun Xiao

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

class Communication {
private:
    ros::NodeHandle nh_;
    std::string vehicle_type_;
    std::string vehicle_id_;
    geometry_msgs::Point current_position_;
    double current_yaw_;
    int hover_flag_;
    int coordinate_frame_;
    mavros_msgs::PositionTarget target_motion_;
    bool arm_state_;
    int motion_type_;
    std::string flight_mode_;
    std::string mission_;
    std::string last_cmd_;
    ros::Publisher target_motion_pub_;
    ros::Subscriber local_pose_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber cmd_pose_flu_sub_;
    ros::Subscriber cmd_pose_enu_sub_;
    ros::Subscriber cmd_vel_flu_sub_;
    ros::Subscriber cmd_vel_enu_sub_;
    ros::Subscriber cmd_accel_flu_sub_;
    ros::Subscriber cmd_accel_enu_sub_;
    ros::ServiceClient armService_;
    ros::ServiceClient flightModeService_;
    
public:
    Communication(const std::string& vehicle_type, const std::string& vehicle_id) :
        vehicle_type_(vehicle_type),
        vehicle_id_(vehicle_id)
    {
        current_yaw_ = 0;
        hover_flag_ = 0;
        coordinate_frame_ = 1;
        target_motion_.coordinate_frame = coordinate_frame_;
        arm_state_ = false;
        motion_type_ = 0;
        flight_mode_ = "";
        mission_ = "";
        last_cmd_ = "";

        target_motion_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(vehicle_type_ + "_" + vehicle_id_ + "/mavros/setpoint_raw/local", 1);
        
        local_pose_sub_ = nh_.subscribe(vehicle_type_ + "_" + vehicle_id_ + "/mavros/local_position/pose", 1, &Communication::local_pose_callback, this);
        cmd_sub_ = nh_.subscribe("/xtdrone/" + vehicle_type_ + "_" + vehicle_id_ + "/cmd", 3, &Communication::cmd_callback, this);
        cmd_pose_flu_sub_ = nh_.subscribe("/xtdrone/" + vehicle_type_ + "_" + vehicle_id_ + "/cmd_pose_flu", 1, &Communication::cmd_pose_flu_callback, this);
        cmd_pose_enu_sub_ = nh_.subscribe("/xtdrone/" + vehicle_type_ + "_" + vehicle_id_ + "/cmd_pose_enu", 1, &Communication::cmd_pose_enu_callback, this);
        cmd_vel_flu_sub_ = nh_.subscribe("/xtdrone/" + vehicle_type_ + "_" + vehicle_id_ + "/cmd_vel_flu", 1, &Communication::cmd_vel_flu_callback, this);
        cmd_vel_enu_sub_ = nh_.subscribe("/xtdrone/" + vehicle_type_ + "_" + vehicle_id_ + "/cmd_vel_enu", 1, &Communication::cmd_vel_enu_callback, this);
        cmd_accel_flu_sub_ = nh_.subscribe("/xtdrone/" + vehicle_type_ + "_" + vehicle_id_ + "/cmd_accel_flu", 1, &Communication::cmd_accel_flu_callback, this);
        cmd_accel_enu_sub_ = nh_.subscribe("/xtdrone/" + vehicle_type_ + "_" + vehicle_id_ + "/cmd_accel_enu", 1, &Communication::cmd_accel_enu_callback, this);
        
        armService_ = nh_.serviceClient<mavros_msgs::CommandBool>(vehicle_type_ + "_" + vehicle_id_ + "/mavros/cmd/arming");
        flightModeService_ = nh_.serviceClient<mavros_msgs::SetMode>(vehicle_type_ + "_" + vehicle_id_ + "/mavros/set_mode");
        
        ROS_INFO("%s: communication initialized", (vehicle_type_ + "_" + vehicle_id_).c_str());
    }

    void run() {
        ros::Rate rate(50);
        while (ros::ok()) {
            target_motion_pub_.publish(target_motion_);
            ros::spinOnce();  // Added to handle callbacks
            rate.sleep();
        }
    }

    void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_position_ = msg->pose.position;
        q2yaw(current_yaw_, msg->pose.orientation);
    }

    void construct_target(mavros_msgs::PositionTarget& output_target_raw, 
                            const double& x, const double& y, const double& z, 
                            const double& vx, const double& vy, const double& vz, 
                            const double& afx, const double& afy, const double& afz, 
                            const double& yaw, const double& yaw_rate) 
    {
        output_target_raw.coordinate_frame = coordinate_frame_;

        output_target_raw.position.x = x;
        output_target_raw.position.y = y;
        output_target_raw.position.z = z;

        output_target_raw.velocity.x = vx;
        output_target_raw.velocity.y = vy;
        output_target_raw.velocity.z = vz;

        output_target_raw.acceleration_or_force.x = afx;
        output_target_raw.acceleration_or_force.y = afy;
        output_target_raw.acceleration_or_force.z = afz;

        output_target_raw.yaw = yaw;
        output_target_raw.yaw_rate = yaw_rate;

        if (motion_type_ == 0) {
            output_target_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY + mavros_msgs::PositionTarget::IGNORE_VZ
                + mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ
                + mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        }
        if (motion_type_ == 1) {
            output_target_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_PX + mavros_msgs::PositionTarget::IGNORE_PY + mavros_msgs::PositionTarget::IGNORE_PZ
                + mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ
                + mavros_msgs::PositionTarget::IGNORE_YAW;
        }
        if (motion_type_ == 2) {
            output_target_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_PX + mavros_msgs::PositionTarget::IGNORE_PY + mavros_msgs::PositionTarget::IGNORE_PZ
                + mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY + mavros_msgs::PositionTarget::IGNORE_VZ
                + mavros_msgs::PositionTarget::IGNORE_YAW;
        }
    }

    void cmd_pose_flu_callback(const geometry_msgs::Pose::ConstPtr& msg) {
        coordinate_frame_ = 9;
        motion_type_ = 0;
        q2yaw(current_yaw_, msg->orientation);
        construct_target(target_motion_, msg->position.x, msg->position.y, msg->position.z, 0, 0, 0, 0, 0, 0, current_yaw_, 0);
    }

    void cmd_pose_enu_callback(const geometry_msgs::Pose::ConstPtr& msg) {
        coordinate_frame_ = 1;
        motion_type_ = 0;
        q2yaw(current_yaw_, msg->orientation);
        construct_target(target_motion_, msg->position.x, msg->position.y, msg->position.z, 0, 0, 0, 0, 0, 0, current_yaw_, 0);
    }

    void cmd_vel_flu_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        hover_state_transition(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
        if (hover_flag_ == 0) {
            coordinate_frame_ = 8;
            motion_type_ = 1;
            construct_target(target_motion_, 0, 0, 0, msg->linear.x, msg->linear.y, msg->linear.z, 0, 0, 0, 0, msg->angular.z);
        }
    }

    void cmd_vel_enu_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        hover_state_transition(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
        if (hover_flag_ == 0) {
            coordinate_frame_ = 1;
            motion_type_ = 1;
            construct_target(target_motion_, 0, 0, 0, msg->linear.x, msg->linear.y, msg->linear.z, 0, 0, 0, 0, msg->angular.z);
        }
    }

    void cmd_accel_flu_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        hover_state_transition(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
        if (hover_flag_ == 0) {
            coordinate_frame_ = 8;
            motion_type_ = 2;
            construct_target(target_motion_, 0, 0, 0, 0, 0, 0, msg->linear.x, msg->linear.y, msg->linear.z, 0, msg->angular.z);
        }
    }

    void cmd_accel_enu_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        hover_state_transition(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
        if (hover_flag_ == 0) {
            coordinate_frame_ = 1;
            motion_type_ = 2;
            construct_target(target_motion_, 0, 0, 0, 0, 0, 0, msg->linear.x, msg->linear.y, msg->linear.z, 0, msg->angular.z);
        }
    }

    void hover_state_transition(const double& x,const double& y,const double& z,const double& w) {
        if (fabs(x) > 0.02 || fabs(y) > 0.02 || fabs(z) > 0.02 || fabs(w) > 0.005) {
            hover_flag_ = 0;
            flight_mode_ = "OFFBOARD";
        }
        else if (flight_mode_ != "HOVER") {
            hover_flag_ = 1;
            flight_mode_ = "HOVER";
            hover();
        }
    }

    void cmd_callback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == last_cmd_ || msg->data == "" || msg->data == "stop controlling") {
            return;
        }
        else if (msg->data == "ARM") {
            arm_state_ = arm();

            ROS_INFO("%s_%s: Armed %d", vehicle_type_.c_str(), vehicle_id_.c_str(), arm_state_);
        }
        else if (msg->data == "DISARM") {
            arm_state_ = !disarm();
            ROS_INFO("%s_%s: Armed %d", vehicle_type_.c_str(), vehicle_id_.c_str(), arm_state_);
        }
        else if (msg->data.substr(0, 7) == "mission" && msg->data != mission_) {
            mission_ = msg->data;
            ROS_INFO("%s_%s: %s", vehicle_type_.c_str(), vehicle_id_.c_str(), msg->data.c_str());
        }
        else {
            flight_mode_ = msg->data;
            flight_mode_switch();
        }
        last_cmd_ = msg->data;
    }

    void q2yaw(double& output_yaw, const geometry_msgs::Quaternion& orientation) {
        tf2::Quaternion tf2_quaternion;
        tf2::fromMsg(orientation, tf2_quaternion);
        tf2::Matrix3x3 mat(tf2_quaternion);
        double roll, pitch;
        mat.getRPY(roll, pitch, output_yaw);
    }

    bool arm() {
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (armService_.call(srv)) {
            return true;
        }
        else {
            ROS_ERROR("%s_%s: arming failed!", vehicle_type_.c_str(), vehicle_id_.c_str());
            return false;
        }
    }

    bool disarm() {
        mavros_msgs::CommandBool srv;
        srv.request.value = false;
        if (armService_.call(srv)) {
            return true;
        }
        else {
            ROS_ERROR("%s_%s: disarming failed!", vehicle_type_.c_str(), vehicle_id_.c_str());
            return false;
        }
    }

    void hover() {
        coordinate_frame_ = 1;
        motion_type_ = 0;
        construct_target(target_motion_, current_position_.x, current_position_.y, current_position_.z, 0, 0, 0, 0, 0, 0, current_yaw_, 0);
        ROS_INFO("%s_%s: %s", vehicle_type_.c_str(), vehicle_id_.c_str(), flight_mode_.c_str());
    }

    void flight_mode_switch() {
        if (flight_mode_ == "HOVER") {
            hover_flag_ = 1;
            hover();
        }
        else {
            mavros_msgs::SetMode srv;
            srv.request.custom_mode = flight_mode_;
            if (flightModeService_.call(srv)) {
                ROS_INFO("%s_%s: %s", vehicle_type_.c_str(), vehicle_id_.c_str(), flight_mode_.c_str());
            }
            else {
                ROS_ERROR("%s_%s: %s failed", vehicle_type_.c_str(), vehicle_id_.c_str(), flight_mode_.c_str());
            }
        }
    }

};

int main(int argc, char **argv) {
    if (argc < 3) {
        ROS_ERROR("Usage: rosrun <package_name> <executable_name> <vehicle_type> <vehicle_id>");
        return -1;
    }
    ros::init(argc, argv, std::string(argv[1]) + "_" + std::string(argv[2]) + "_communication");
    Communication communication(argv[1], argv[2]);
    communication.run();
    return 0;
}
