#!/usr/bin/python
import rospy
import roslib
import numpy as np
import geometry_msgs.msg as geo_msg
import std_msgs.msg as std_msg
import time
from std_msgs.msg import Bool
from Mission_Management.msg import my_msg
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_matrix
from bhand_controller.msg import State, TactileArray, Service, ForceTorque
from bhand_controller.srv import Actions, SetControlMode
from URX.srv import moveUR, desiredTCP, fireDrill

class MissionManagement:
    def __init__(self):
        self.ros_node = rospy.init_node('3DVS_Manager', anonymous=True)

        #define ROS Publisher and Subscriber
        self.pose_ur_pub = rospy.Publisher('/ur_cmd_pose', geo_msg.Pose, queue_size=1 )
        self.adjust_pose_ur_pub = rospy.Publisher('/ur_cmd_adjust_pose', geo_msg.Pose, queue_size=1 )
        self.vel_ur_pub  = rospy.Publisher('/ur_cmd_vel', geo_msg.Twist, queue_size=1 )
        self.Hole_Subscriber = rospy.Subscriber("/hole_pos", geo_msg.Point, self.centroid_Pose_callback)
        #self.icp_Quat_Subscriber = rospy.Subscriber("/icp_Quat", geo_msg.Quaternion, self.Plane_Orien_callback)
        self.detection_mode_pub = rospy.Publisher('/detection_mode', Bool, queue_size=1)
        self.tactile_mode_pub = rospy.Publisher('/tactile_control_mode', Bool, queue_size=1)

        self.rate = rospy.Rate(100)

        #define ROS service
        self.MM_service_start = rospy.Service('startMM', SetBool, self.start_MM)
        self.startEMVScall = rospy.ServiceProxy('startEMVS', SetBool)
        self.move_UR = rospy.ServiceProxy('move_ur', moveUR)
        self.setBHand_controlmode_call = rospy.ServiceProxy('/bhand_node/set_control_mode', SetControlMode)
        self.BHand_actions_call = rospy.ServiceProxy('/bhand_node/actions', Actions)
        self.hole_pose = []
        self.plane_orientation = np.empty((4,))  

        self.tactile_adjusted_pose = geo_msg.Pose()
        
        #define BHand parameters
        self.INIT_HAND = 1
        self.CLOSE_GRASP = 2
        self.OPEN_GRASP = 3
        self.MIDDLE_SPREAD = 4
        self.CLOSE_2_FINGERS = 5
        self.HALF_CLOSE_GRASP = 6

        #CAM Pose
        self.camera_final_pose = geo_msg.Pose()

        rospy.spin()

    def start_MM(self, mess):
        if mess.data == True:
            print("MissionManagment Started")       
            
            # Intial Pose
            self.camera_final_pose.position.x = 0.2929
            self.camera_final_pose.position.y = -0.4907
            self.camera_final_pose.position.z = 0.5956
            self.camera_final_pose.orientation.x = 0.0
            self.camera_final_pose.orientation.y = 0.9841
            self.camera_final_pose.orientation.z = -0.1772
            self.camera_final_pose.orientation.w = 0.0
            self.move_UR("davis", self.camera_final_pose)
            
            startEMVS = self.startEMVScall(True)
            time.sleep(2)

            # Final Pose
            camera_initial_pose = geo_msg.Pose()
            camera_initial_pose.position.x = -0.0129
            camera_initial_pose.position.y = -0.4907
            camera_initial_pose.position.z = 0.5956
            camera_initial_pose.orientation.x = 0.0
            camera_initial_pose.orientation.y = 0.9841
            camera_initial_pose.orientation.z = -0.1772
            camera_initial_pose.orientation.w = 0.0
            desired_pose = moveUR()
            self.move_UR("davis", camera_initial_pose)
            
            # #Velocity Command
            # camera_vel_cmd = geo_msg.Twist()
            # camera_vel_cmd.linear.x = -0.1
            # camera_vel_cmd.linear.y = 0
            # camera_vel_cmd.linear.z = 0
            # camera_vel_cmd.angular.x = 0
            # camera_vel_cmd.angular.y = 0
            # camera_vel_cmd.angular.z = 0
            # self.vel_ur_pub.publish(camera_vel_cmd)

    def centroid_Pose_callback(self, data):
        print(data)
        #self.Top_Down_Grasp(data)
    
    def Top_Down_Grasp(self, data):
        print("Top Down Grasp Initiated")
        #Pose Command
        y_correction = -0.06
        davis_to_Bhand_distance = - 0.230365
        z_clearance = 0.15
        Top_Down_Grasp_Pose = geo_msg.Pose()
        Top_Down_Grasp_Pose.position.x = data.x + 0.02
        Top_Down_Grasp_Pose.position.y = data.y + y_correction + davis_to_Bhand_distance
        Top_Down_Grasp_Pose.position.z = data.z + z_clearance
        Top_Down_Grasp_Pose.orientation.x = 0.0
        Top_Down_Grasp_Pose.orientation.y = 1.0
        Top_Down_Grasp_Pose.orientation.z = 0.0
        Top_Down_Grasp_Pose.orientation.w = 0.0
        desired_pose = moveUR()
        self.move_UR("davis", Top_Down_Grasp_Pose)

        #Initiate Hand and Close the Grasp
        Inialize_hand = self.BHand_actions_call(self.INIT_HAND)
        time.sleep(5)
        close_grasp = self.BHand_actions_call(self.CLOSE_GRASP)

    def Side_Grasp(self, data):
        print("Side Grasp Initiated")

        #Orientation Command
        Side_Grasp = geo_msg.Pose()
        Side_Grasp.position.x = self.camera_final_pose.position.x
        Side_Grasp.position.y = self.camera_final_pose.position.y
        Side_Grasp.position.z = self.camera_final_pose.position.z
        Side_Grasp.orientation.x = 0.0
        Side_Grasp.orientation.y = 0.7
        Side_Grasp.orientation.z = -0.7
        Side_Grasp.orientation.w = 0.0
        desired_pose = moveUR()
        self.move_UR("davis", Side_Grasp)
        
        #Initiate Hand and Close the Grasp
        Inialize_hand = self.BHand_actions_call(self.INIT_HAND)
        time.sleep(5)
        open_grasp = self.BHand_actions_call(self.OPEN_GRASP)
        MIDDLE_SPREAD = self.BHand_actions_call(self.MIDDLE_SPREAD)
        # time.sleep(3)

        #Position Command
        y_correction = 0
        davis_to_Bhand_distance = 0.230365
        z_clearance = 0.07
        Side_Grasp.position.x = data.x + 0.01
        Side_Grasp.position.y = data.y + y_correction 
        Side_Grasp.position.z = data.z + z_clearance + davis_to_Bhand_distance
        print(Side_Grasp)
        desired_pose = moveUR()
        self.move_UR("davis", Side_Grasp)

        #Grasp With Two Fingers
        close_two_Fingers = self.BHand_actions_call(self.CLOSE_2_FINGERS)
        time.sleep(2)
        Side_Grasp.position.z = Side_Grasp.position.z + 0.2
        desired_pose = moveUR()
        self.move_UR("davis", Side_Grasp)



    # def AdjustDepth(self, plane_Quat, depth_vector_plane):
    #     Quat = [plane_Quat[0], plane_Quat[1], plane_Quat[2], plane_Quat[3]]
        
    #     PlanetoInertialRotMat = R.from_quat([Quat])
    #     print(np.matmul(PlanetoInertialRotMat.as_dcm(), depth_vector_plane))
    #     return np.matmul(PlanetoInertialRotMat.as_dcm(), depth_vector_plane)
      
    # def Start_2DVS(self):
    #     start_VS = Bool()
    #     start_VS.data = True 
    #     self.detection_mode_pub.publish(start_VS)
    #     try:
    #         Servoing_complete = rospy.wait_for_message('ur_detection_status', Bool, timeout = 10.0)
    #     except:
    #         start_VS = Bool()
    #         start_VS.data = False 
    #         self.detection_mode_pub.publish(start_VS)
            
    #     start_VS = Bool()
    #     start_VS.data = False 
    #     self.detection_mode_pub.publish(start_VS)
    #     #print("Servoing Complete: %d", Servoing_complete)

    # def Adjust_tool_position(self):
    #     x_correction = -0.0043
    #     y_correction = -0.0521
    #     z_correction = 0 
    #     Adjust_pose = geo_msg.Pose()
    #     Adjust_pose.position.x = x_correction
    #     Adjust_pose.position.y = y_correction
    #     Adjust_pose.position.z = z_correction
    #     Adjust_pose.orientation.x = 0
    #     Adjust_pose.orientation.y = 0
    #     Adjust_pose.orientation.z = 0
    #     Adjust_pose.orientation.w = 1
    #     self.adjust_pose_ur_pub.publish(Adjust_pose)

if __name__ == '__main__':
    MissionManagement()
    exit()  


