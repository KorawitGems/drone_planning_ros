#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Eigen>

#include <cmath>
#include <thread>
#include <mutex>
#include <memory>

class SamplingLocalToController
{
public:
    SamplingLocalToController(const std::string &vehicle_type, const std::string &vehicle_id)
      : vehicle_type_(vehicle_type), vehicle_id_(vehicle_id),
        inflated_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
        kdtree_(new pcl::search::KdTree<pcl::PointXYZ>())
    {
        current_pose_sub_ = nh_.subscribe("current_pose", 10, &SamplingLocalToController::currentPoseCallback, this);
        path_sub_ = nh_.subscribe("path_planning", 10, &SamplingLocalToController::localPathCallback, this);
        inflated_cloud_sub_ = nh_.subscribe("inflated_cloud_map", 50, &SamplingLocalToController::inflatedCloudCallback, this);
        goal_sub_ = nh_.subscribe("/local_planner/goal", 10, &SamplingLocalToController::localGoalCallback, this);
        global_goal_sub_ = nh_.subscribe("/command/goal", 10, &SamplingLocalToController::globalGoalCallback, this);
        loop_publisher_timer_ = nh_.createTimer(ros::Duration(0.02), &SamplingLocalToController::loopPublisherCallback, this);

        pose_cmd_pub_ = nh_.advertise<geometry_msgs::Pose>("/xtdrone/" + vehicle_type_ + "_" + vehicle_id_ + "/cmd_pose_enu", 1);
        vel_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/xtdrone/" + vehicle_type_ + "_" + vehicle_id_ + "/cmd_vel_flu", 1);

        nh_.param("/local_planner/replan", param_local_replan_, false);
        nh_.param("/local_planner/sampling_local_planner/collision_check/collision_radius", param_collision_radius_, 0.4);
        ROS_WARN("[local_to_controller] Param: collision_radius: %f", param_collision_radius_);
        param_local_recovery_mode_ = false;
        param_local_receive_plan_ = false;
        last_time_local_point_ = ros::Time::now();
        last_time_local_replan_ = ros::Time::now();
        timeout_step_point_ = 1.0;
        reserve_local_points_.emplace_back(geometry_msgs::Pose());
        last_k_distance_ = 0.2;
    }

    ~SamplingLocalToController() 
    {}

    void globalGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &global_goal_msg)
    {
        global_goal_ = *global_goal_msg;
    }

    void localGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &local_goal_msg)
    {
        local_goal_ = *local_goal_msg;
    }

    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        current_pose_ = *pose_msg;
    }

    void localPathCallback(const nav_msgs::Path::ConstPtr &path_msg)
    {
        local_points_.clear();
        for (const auto &pose : path_msg->poses)
        {
            local_points_.emplace_back(pose.pose);
        }
        if (local_points_.size() > 1) 
            {
                local_points_.erase(local_points_.begin());
            }
        param_local_receive_plan_ = true;
        nh_.setParam("/local_planner/replan", false);
        last_time_local_point_ = ros::Time::now() - ros::Duration(timeout_step_point_);

    }

    void inflatedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) 
    {

        pcl::fromROSMsg(*cloud_msg, *inflated_cloud_);
    }

    bool isInInflatedCloud(const geometry_msgs::Point& point_position, 
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_inflated_cloud,
                            const float& radius_threshold) 
    {
        if (ptr_inflated_cloud->empty()) {
            // ROS_WARN("Inflated cloud is empty. Cannot check collision.");
            return false;
        }

        pcl::PointXYZ main_point(point_position.x, point_position.y, point_position.z);
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        kdtree_->setInputCloud(ptr_inflated_cloud);

        if (kdtree_->radiusSearch(main_point, radius_threshold, k_indices, k_sqr_distances, 1) > 0)
        {
            k_distance_ = std::sqrt(k_sqr_distances[0]);
            obstacle_point_.x = ptr_inflated_cloud->points[k_indices[0]].x;
            obstacle_point_.y = ptr_inflated_cloud->points[k_indices[0]].y;
            obstacle_point_.z = ptr_inflated_cloud->points[k_indices[0]].z;
            // ROS_INFO("position (%f,%f,%f) is the distance %f from a point cloud (%f,%f,%f)",
            //             pcl_point.x, pcl_point.y, pcl_point.z, k_distance,
            //             ptr_inflated_cloud->points[k_indices[0]].x, ptr_inflated_cloud->points[k_indices[0]].y,
            //             ptr_inflated_cloud->points[k_indices[0]].z);
            return true;
        }
        return false;
    }

    void publishPoseToController(const geometry_msgs::Pose& current_local_point)
    {
        nh_.getParam("/planner_to_mavros/state", param_planner_to_mavros_);
        if (param_planner_to_mavros_)
        {
            geometry_msgs::Pose pose_enu;
            tf2::Quaternion quaternion;
            double yaw;
            // set tolerance small to neglect local goal's orientation
            if (calculateDistance(current_pose_.pose, global_goal_.pose) < 1.0)
            {
                pose_enu.orientation = global_goal_.pose.orientation;
                //yaw = getYawFromQuaternion(local_goal_.pose.orientation);
            }
            else
            {
                // nh_.getParam("/planner_to_mavros/restrict_yaw/state", param_restrict_yaw_state_);
                // if (param_restrict_yaw_state_)
                // {
                //     nh_.getParam("/planner_to_mavros/restrict_yaw/value", param_restrict_yaw_value_);
                //     yaw = param_restrict_yaw_value_;
                // }
                // else
                // {
                //     // Calculate orientation to align the robot's heading towards the goal point
                //     yaw = atan2(current_local_point.position.y - current_pose_.pose.position.y,
                //                 current_local_point.position.x - current_pose_.pose.position.x);
                // }
                yaw = atan2(current_local_point.position.y - current_pose_.pose.position.y,
                                current_local_point.position.x - current_pose_.pose.position.x);
                quaternion.setRPY(0, 0, yaw);
                pose_enu.orientation.x = quaternion.x();
                pose_enu.orientation.y = quaternion.y();
                pose_enu.orientation.z = quaternion.z();
                pose_enu.orientation.w = quaternion.w();
            }

            pose_enu.position = current_local_point.position;
            pose_cmd_pub_.publish(pose_enu);
            ROS_INFO("publish local point to mavros: position (%f, %f, %f) yaw %f", pose_enu.position.x, pose_enu.position.y, pose_enu.position.z, yaw);
        }
        else
        {
            vel_cmd_pub_.publish(geometry_msgs::Twist());
            ROS_WARN("Can not send points from local planner to mavros due to param /planner_to_mavros/state : %d.", param_planner_to_mavros_);
        }
    }

    double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion)
    {
        tf2::Quaternion tf_quaternion(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

        return yaw;
    }

    double calculateDistance(const  geometry_msgs::Pose& current_pose, 
                                const  geometry_msgs::Pose& goal_pose)
    {
        return std::sqrt(std::pow(current_pose.position.x - goal_pose.position.x, 2) +
                        std::pow(current_pose.position.y - goal_pose.position.y, 2) +
                        std::pow(current_pose.position.z - goal_pose.position.z, 2));
    }

    void prepareLocalToController()
    {
        if (!local_points_.empty())
        {
            // check to not repeat publish local point too often
            if ( (ros::Time::now() - last_time_local_point_) > ros::Duration(timeout_step_point_) )
            {
                //ROS_INFO("(ros::Time::now() - last_time_local_point_) > ros::Duration(timeout_step_point_)");
                publishPoseToController(local_points_[0]);
                last_time_local_point_ = ros::Time::now();
            }
            // high level control local point follow local path
            if (calculateDistance(current_pose_.pose, local_points_[0]) < 1.0) 
            {
                //vel_cmd_pub_.publish(geometry_msgs::Twist());
                ROS_INFO("Successfully achieve local point (%f,%f,%f)", 
                            local_points_[0].position.x, local_points_[0].position.y, 
                            local_points_[0].position.z );
                reserve_local_points_ = local_points_;
                local_points_.erase(local_points_.begin());
                last_time_local_point_ = ros::Time::now() - ros::Duration(timeout_step_point_);
            }
        }
    }

    // publish loop
    void loopPublisherCallback(const ros::TimerEvent& event)
    {
        if (!param_local_recovery_mode_) // Is current pose in local collision radius?
        {
            if (isInInflatedCloud(current_pose_.pose.position, inflated_cloud_, param_collision_radius_ - 0.05))
            {
                vel_cmd_pub_.publish(geometry_msgs::Twist());
                param_local_recovery_mode_ = true;
                param_local_receive_plan_ = false;
                nh_.setParam("/local_planner/replan", true);
                ROS_INFO("Robot is\033[1;31m distance %f\033[0m from inflated cloud.", (k_distance_));
                ROS_INFO("\033[1;32m << Recovery Mode >> \033[0m");
            }
        }

        if (!local_points_.empty()) // Is local path in local collision radius?
        {
            for (int i = 0; i <= 5 && i < local_points_.size(); ++i) 
            {
                if (isInInflatedCloud(local_points_[i].position, inflated_cloud_, param_collision_radius_ - 0.05)) 
                {
                    local_points_.clear();
                    vel_cmd_pub_.publish(geometry_msgs::Twist());
                    nh_.setParam("/local_planner/replan", true);
                    param_local_receive_plan_ = false;
                    ROS_WARN("Local goal point is too close the obstacle_position. Replan local path planning");
                    last_time_local_replan_ = ros::Time::now();
                }
            }
        }

        if (param_local_receive_plan_)
        {
            param_local_recovery_mode_ = false;
            prepareLocalToController();
        }
    }

    // run main
    void run()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber current_pose_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber inflated_cloud_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber global_goal_sub_;
    ros::Publisher pose_cmd_pub_;
    ros::Publisher vel_cmd_pub_;
    ros::Timer loop_publisher_timer_;

    std::string vehicle_type_;
    std::string vehicle_id_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inflated_cloud_;
    geometry_msgs::Point obstacle_point_;
    geometry_msgs::PoseStamped global_goal_;
    geometry_msgs::PoseStamped local_goal_;
    geometry_msgs::PoseStamped current_pose_;
    std::vector<geometry_msgs::Pose> local_points_;
    std::vector<geometry_msgs::Pose> reserve_local_points_;

    bool param_planner_to_mavros_;
    bool param_restrict_yaw_state_;
    float param_restrict_yaw_value_;
    bool param_local_replan_;
    bool param_local_receive_plan_;
    bool param_local_recovery_mode_;
    double param_collision_radius_;
    float k_distance_;
    float last_k_distance_;
    ros::Time last_time_local_point_;
    ros::Time last_time_local_replan_;
    float timeout_step_point_;
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "local_to_controller");

    if (argc != 3)
    {
        ROS_ERROR("Usage: %s <vehicle_type> <vehicle_id>", argv[0]);
        return 1;
    }

    SamplingLocalToController local_to_controller(argv[1], argv[2]);
    local_to_controller.run();

    return 0;
}