#!/usr/bin/env python3
from __future__ import division
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import math

from tf.transformations import quaternion_from_euler

class Goal:
    
    def __init__(self,vehicles,ids):
        self.vehicle_ = vehicles
        self.id_ = int(ids)
        self.pub_goal_ = rospy.Publisher("/command/goal",PoseStamped,queue_size=1)

        self.sub_current_pose_ = rospy.Subscriber("/current_pose",PoseStamped,self.Current_Pose_Callback,queue_size=10)
        self.sub_stop_ = rospy.Subscriber("/mandatory_stop", Empty, self.Stop_Callback, queue_size=10)
        self.sub_detected_object_ = rospy.Subscriber('/object_detection/marker_array/single', MarkerArray,self.Object_Detection_Callback, queue_size=50)
        
        self.current_pose_ = PoseStamped()
        self.goal_point_ = PoseStamped()
        self.planner_to_mavros_state_ = rospy.get_param('/planner_to_mavros/state', True)
        self.restrict_yaw_state_ = rospy.get_param('/planner_to_mavros/restrict_yaw/state', False)
        self.restrict_yaw_value_ = rospy.get_param('/planner_to_mavros/restrict_yaw/value', 0.0)
        self.replan_state_ = rospy.get_param('/global_planner/replan', False)
        self.last_time_global_param_ = rospy.Time.now()
        #self.goals_list_1_ = [[26.0,-3.0,2.8,math.pi],[25,-10.0,3.0,math.pi],[24.50,-18.0,2.8,math.pi/2]]
        self.goals_list_1_ = [[16.6,16.6,3.0,-math.pi/2],[20.0,-20.0,3.5,math.pi/2],[10.0,5.0,2.0,-math.pi/2]]
        #self.goals_list_1_ = [[1.0,2.0,3.0,0.0],[2.0,1.0,3.5,0.0]]
        self.goals_list_2_ = [[25,-27.0,3.5,-math.pi/2],[23.0,-35.0,3.5,math.pi],[10.0,-34.0,3.1,math.pi/2],[12.0,-26.0,3.2,0.0]]
        self.goals_list_ = self.goals_list_1_
        self.detected_state_ = False
        self.detected_zone_ = 1
        self.plan_time_up_state_ = rospy.get_param('/global_planner/sampling_global_planner/plan/time_up', False)

    def Object_Detection_Callback(self,object_markers_msg):
        #rospy.loginfo(object_markers_msg)
        if self.detected_zone_ == 1:
            for marker in object_markers_msg.markers:
                if marker.ns == "chair": # "bicycle"or  marker.ns == "fire hydrant" or  marker.ns == "bottle":
                    rospy.set_param('/planner_to_mavros/state', False)
                    self.goals_list_ = self.goals_list_2_
                    rospy.logwarn("Move to Zone 2")
                    self.detected_state_ = True
                    self.detected_zone_ = 2
        else:
            for marker in object_markers_msg.markers:
                if  marker.ns == "car": #or  marker.ns == "truck":
                    self.goals_list_ = [[0.0,0.0,3.2,0.0],[-1.0,-1.0,3.0,0.0]]
                    rospy.logwarn("Go back to home")
                    self.detected_state_ = True
                    self.detected_zone_ = 1
        
    def Publish_Goal(self,goal,yaw):
        self.goal_point_.header.frame_id = "map"
        self.goal_point_.header.stamp = rospy.Time.now()
        self.goal_point_.pose.position.x = goal[0]
        self.goal_point_.pose.position.y = goal[1]
        self.goal_point_.pose.position.z = goal[2]
        quat = quaternion_from_euler(0, 0, yaw)
        self.goal_point_.pose.orientation.x = quat[0]
        self.goal_point_.pose.orientation.y = quat[1]
        self.goal_point_.pose.orientation.z = quat[2]
        self.goal_point_.pose.orientation.w = quat[3]
        self.pub_goal_.publish(self.goal_point_)

    def Current_Pose_Callback(self,pose_msg):
        self.current_pose_ = pose_msg

    def Stop_Callback(self,stop_msg):
        pass
        #rospy.logwarn("Stop assign goal")

    def calculate_distance(self,current_position,goal_position):
        return math.sqrt((goal_position.pose.position.x - current_position.pose.position.x)**2 +
                        (goal_position.pose.position.y - current_position.pose.position.y)**2 +
                        (goal_position.pose.position.z - current_position.pose.position.z)**2)

    def Check_Publish(self,goal,yaw):
        self.planner_to_mavros_state_ = rospy.get_param('/planner_to_mavros/state', False)
        if self.planner_to_mavros_state_:
            rospy.loginfo("publish goal plan to global planner x:%s y:%s z:%s" % (goal[0], goal[1], goal[2]))
            self.Publish_Goal(goal,yaw)
        else:
            rospy.logwarn("Can not assign goal to global planner due to param /planner_to_mavros/state : %s." % (self.planner_to_mavros_state_))
                    
    def Run(self):
        rospy.sleep(3)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.detected_state_ = False
            for x, y, z, yaw in self.goals_list_:
                self.restrict_yaw_value_ = rospy.set_param('/planner_to_mavros/restrict_yaw/value', yaw)
                goal = [x, y, z]
                self.Check_Publish(goal,yaw)
                
                last_time = rospy.Time.now() - rospy.Duration(5.0)
                last_pose = self.current_pose_
                goal_distance = self.calculate_distance(self.current_pose_,self.goal_point_)
                # last_time_global_point = rospy.Time.now()
                while ( goal_distance > 0.1 ):
                    #rospy.loginfo("while ( goal_distance %s > 0.05 )"% (goal_distance))
                    # if ( rospy.Time.now() - last_time_global_point ) > rospy.Duration(30.0): # time up for each global point
                    #     rospy.logwarn("Time out to reach global point")
                    #     break
                    self.plan_time_up_state_ = rospy.get_param('/global_planner/sampling_global_planner/plan/time_up', False)
                    if self.plan_time_up_state_:
                        rospy.logwarn("Time up to get global plan. Skip goal point")
                        rospy.set_param('/global_planner/sampling_global_planner/plan/time_up', False)
                        last_pose = self.current_pose_
                        last_time = rospy.Time.now()
                        break

                    if self.detected_state_ == True:
                        last_time_detect_object = rospy.Time.now()
                        rospy.logwarn("Stop about 3 sec")
                        while ( rospy.Time.now() - last_time_detect_object ) < rospy.Duration(3.0):
                            rospy.Rate(1).sleep()
                        rospy.set_param('/planner_to_mavros/state', True)
                        break

                    self.replan_state_ = rospy.get_param('/global_planner/replan', False)
                    if self.replan_state_:
                        rospy.loginfo("Receive global replan from global to local")
                        rospy.set_param('/global_planner/replan', False)
                        self.Check_Publish(goal,yaw)
                        last_pose = self.current_pose_
                        last_time = rospy.Time.now()


                    if ( rospy.Time.now() - last_time ) > rospy.Duration(3.0):
                        stationary_distance = self.calculate_distance(self.current_pose_,last_pose)
                        if ( stationary_distance < 0.2 ):
                            rospy.loginfo("Replan due to stop moving for %s sec" % (rospy.Duration(3.0)))
                            self.Check_Publish(goal,yaw)
                            last_pose = self.current_pose_
                        last_time = rospy.Time.now()
                    goal_distance = self.calculate_distance(self.current_pose_,self.goal_point_)
                    rate.sleep()

                if self.detected_state_ == True:
                    rospy.logwarn("Successful detect object")
                    break
            #break

if __name__ == '__main__':
    try:
        rospy.init_node("goal_plan")
        goal_iris_0 = Goal("iris",0)
        goal_iris_0.Run()
    except rospy.ROSInterruptException:
        pass
