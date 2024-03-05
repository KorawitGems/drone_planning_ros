#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <thread>
#include <mutex>
#include <memory>

class SamplingGlobalToLocal 
{
public:
    SamplingGlobalToLocal(const std::string &vehicle_type, const std::string &vehicle_id)
      : vehicle_type_(vehicle_type), vehicle_id_(vehicle_id),
        inflated_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
        kdtree_(new pcl::search::KdTree<pcl::PointXYZ>())
    {
        current_pose_sub_ = nh_.subscribe("current_pose", 10, &SamplingGlobalToLocal::currentPoseCallback, this);
        path_sub_ = nh_.subscribe("path_planning", 10, &SamplingGlobalToLocal::globalPathCallback, this);
        inflated_cloud_sub_ = nh_.subscribe("inflated_cloud_map", 50, &SamplingGlobalToLocal::inflatedCloudCallback, this);
        goal_sub_ = nh_.subscribe("/command/goal", 10, &SamplingGlobalToLocal::globalGoalCallback, this);
        loop_publisher_timer_ = nh_.createTimer(ros::Duration(0.02), &SamplingGlobalToLocal::loopPublisherCallback, this);

        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_planner/goal", 1);

        nh_.param<bool>("/global_planner/replan", param_global_replan_, false);
        nh_.param<double>("/global_planner/sampling_global_planner/collision_check/collision_radius", param_collision_radius_, 0.6);
        ROS_WARN("[global_to_local] Param: collision_radius: %f", param_collision_radius_);
        last_time_global_point_ = ros::Time::now();
        last_time_global_replan_ = ros::Time::now();
        last_time_global_set_param_ = ros::Time::now();
        timeout_step_point_ = 5.0;
        reserve_global_points_.emplace_back(geometry_msgs::Pose());
        param_global_receive_plan_ = false;
    }

    ~SamplingGlobalToLocal() 
    {}

    void globalGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &global_goal_msg)
    {
        global_goal_ = *global_goal_msg;
    }

    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        current_pose_ = *pose_msg;
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr &path_msg)
    {
        global_points_.clear();
        for (const auto &pose : path_msg->poses)
        {
            global_points_.emplace_back(pose.pose);
        }
        if (global_points_.size() > 1)
            {
                global_points_.erase(global_points_.begin());
            }
        param_global_receive_plan_ = true;
        nh_.setParam("/global_planner/replan", false);

        last_time_global_point_ = ros::Time::now() - ros::Duration(timeout_step_point_);

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
            // ROS_INFO("position (%f,%f,%f) is the distance %f from a point cloud (%f,%f,%f)",
            //             pcl_point.x, pcl_point.y, pcl_point.z, k_distance,
            //             ptr_inflated_cloud->points[k_indices[0]].x, ptr_inflated_cloud->points[k_indices[0]].y,
            //             ptr_inflated_cloud->points[k_indices[0]].z);
            return true;
        }
        return false;
    }

    void publishPose(const  geometry_msgs::Pose& current_global_point)
    {
        geometry_msgs::PoseStamped pose_enu_stamp;
        pose_enu_stamp.header.stamp = ros::Time::now();
        pose_enu_stamp.header.frame_id = "map";
        pose_enu_stamp.pose.position = current_global_point.position;
        pose_enu_stamp.pose.orientation = global_goal_.pose.orientation;
        goal_pub_.publish(pose_enu_stamp);
        ROS_INFO("Publishing global point to local planner: position (%f, %f, %f) orientation (%f, %f, %f, %f)", 
                pose_enu_stamp.pose.position.x, pose_enu_stamp.pose.position.y, pose_enu_stamp.pose.position.z, 
                pose_enu_stamp.pose.orientation.x, pose_enu_stamp.pose.orientation.y, pose_enu_stamp.pose.orientation.z, pose_enu_stamp.pose.orientation.w);
    }

    double calculateDistance(const  geometry_msgs::Pose& current_pose, 
                                const  geometry_msgs::Pose& goal_pose)
    {
        return std::sqrt(std::pow(current_pose.position.x - goal_pose.position.x, 2) +
                        std::pow(current_pose.position.y - goal_pose.position.y, 2) +
                        std::pow(current_pose.position.z - goal_pose.position.z, 2));
    }

    void checkGlobalToLocal()
    {
        if (!global_points_.empty())
        {
            // check to not repeat publish global point too often because it will change local point
            if ( (ros::Time::now() - last_time_global_point_) > ros::Duration(timeout_step_point_) )
            {
                publishPose(global_points_[0]);
                last_time_global_point_ = ros::Time::now();
            }
            // high level control local goal(global point) to smooth path
            if (calculateDistance(current_pose_.pose, global_points_[0]) < 2.0) 
            {
                ROS_INFO("Successfully achieve global point (%f,%f,%f)", 
                            global_points_[0].position.x, global_points_[0].position.y, 
                            global_points_[0].position.z );
                reserve_global_points_ = global_points_;
                global_points_.erase(global_points_.begin());
                last_time_global_point_ = ros::Time::now() - ros::Duration(timeout_step_point_); //publish to start local planner
            }

        }
    }

    // publish loop
    void loopPublisherCallback(const ros::TimerEvent& event)
    {
        nh_.getParam("/local_planner/replan", param_local_replan_);
        if (param_local_replan_)
        {
            nh_.setParam("/local_planner/replan", false);
            ROS_INFO("Receive global replan from local to controller");
            nh_.setParam("/global_planner/replan", true);
            param_global_receive_plan_ = false;
            last_time_global_point_ = ros::Time::now() - ros::Duration(timeout_step_point_); // immediately publish global point to replan local point
        }

        if (!global_points_.empty()) 
        {
            for (int i = 0; i <= 7 && i < global_points_.size(); ++i) 
            {
                if (isInInflatedCloud(global_points_[i].position, inflated_cloud_, param_collision_radius_ - 0.05)) 
                {
                    global_points_.clear();
                    nh_.setParam("/global_planner/replan", true);
                    param_global_receive_plan_ = false;
                    ROS_WARN("Global goal point is too close to the obstacle. Replan global path planning");

                }
            }
        }

        if (param_global_receive_plan_)
        {
            checkGlobalToLocal();
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
    ros::Publisher goal_pub_;
    ros::Publisher pose_cmd_pub_;
    ros::Publisher vel_cmd_pub_;
    ros::Timer loop_publisher_timer_;

    std::string vehicle_type_;
    std::string vehicle_id_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inflated_cloud_;
    geometry_msgs::PoseStamped global_goal_;
    geometry_msgs::PoseStamped current_pose_;
    std::vector<geometry_msgs::Pose> global_points_;
    std::vector<geometry_msgs::Pose> reserve_global_points_;

    bool param_planner_to_mavros_;
    bool param_restrict_yaw_state_;
    float param_restrict_yaw_value_;
    bool param_global_replan_;
    bool param_local_replan_;
    bool param_global_receive_plan_;
    double param_collision_radius_;
    float k_distance_;
    ros::Time last_time_global_point_;
    ros::Time last_time_global_replan_;
    ros::Time last_time_global_set_param_;
    float timeout_step_point_;
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "global_to_local");

    if (argc != 3)
    {
        ROS_ERROR("Usage: %s <vehicle_type> <vehicle_id>", argv[0]);
        return 1;
    }

    SamplingGlobalToLocal global_to_local(argv[1], argv[2]);
    global_to_local.run();

    return 0;
}
