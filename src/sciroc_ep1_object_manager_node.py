#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
import os
import sys
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float64
from sciroc_ep1_object_manager.srv import ResetTray
from sciroc_ep1_object_manager.srv import MoveObjectsOnClosesTable
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelProperties
from gazebo_msgs.srv import GetJointProperties
#from beast_srvs.srv import *
#from eurobench_bms_msgs_and_srvs.srv import *





VERBOSE = True



class sciroc_ep1_object_manager:
    def __init__(self):
        print("init")
        #self.cw_left = np.array([None, None, None, None])
		#				   self.ccw_right_callback, queue_size=1)   
        self.robot_pose = [0.0,0.0]		
		
        self.list_of_tables = {
            "cafe_table", 
            "cafe_table_0", 
            "cafe_table_1", 
            "cafe_table_2", 
            "cafe_table_3", 
            "cafe_table_4", 
            "cafe_table_5",
            "cafe_table_"
        }

        self.bank_object = {"table"}				

          
    def startSim(self):
        package = 'eurobench_reemc_cart'
        launch_file = 'reemc_cart.launch'
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
     #   sys.argv = [ 'door:=simple', 'direction:=push', 'robot_placement_cw:=true']
        
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self.launch.start()

        

    def are_ranges_complete(self, ranges):
        for single_range in ranges:
            if single_range == None:
                return False
        return True
        
    def null_the_ranges(self, ranges):
        for single_range in ranges:
            single_range = None
        
    def sensor_identifier(self, ros_data):
        return int(ros_data.header.frame_id[-1])%4

    def publish_strips(self, tmp_ranges, ros_data, ranges_pub):
        sensorID = self.sensor_identifier(ros_data)
        tmp_ranges[sensorID] = ros_data
        if (self.are_ranges_complete(tmp_ranges)):
            msg = Passage()
            #msg.STATUS_OK = 1
            msg.ranges[0] = tmp_ranges[0]
            msg.ranges[1] = tmp_ranges[1]
            msg.ranges[2] = tmp_ranges[2]
            msg.ranges[3] = tmp_ranges[3]
            ranges_pub.publish(msg)
            self.null_the_ranges(tmp_ranges)

    def cw_left_callback(self, ros_data):
        self.publish_strips(self.cw_left, ros_data, self.cw_left_pub)

    def cw_right_callback(self, ros_data):
        self.publish_strips(self.cw_right, ros_data, self.cw_right_pub)
                

    


def talker(ebws):
    if VERBOSE:
        print ("subcribed on sensor_distances")

    r = rospy.Rate(10) #10hz

    msg = Float64()
    
    while not rospy.is_shutdown():
        #msg = getDoorAperture()
        #ebws.door_pub.publish(msg)
        
        get_robot_position()

        #msg_handle = getTrolleyPosition()
        #ebws.door_handle_pub.publish(msg_handle)

        retrieveBenchmarkConfiguration(ebws)
        #if benchmarkConfigurationHasChanged(ebws):
        #    if ebws.current_door_opening_side is not None: 
        #        restartSim(ebws)

        r.sleep()


def callback(data):
    rospy.loginfo("%s is age: %d" % (data.name, data.age))
    print ("initialized")


def reset_tray_srv(req):  
    print("reset_tray_srv service")
    return ResetTray.srvResponse(True, "")

def move_objects_on_the_closest_table_srv(req):  
    print("move_objects_on_the_closest_table_srv service")
    return MoveObjectsOnClosesTable.srvResponse(True, "")
    
    
def is_there_an_object_on(x,y,z):
    return False

def get_scene_object_list(req):
    return []


def set_item(goal_x, goal_y,goal_z, object_to_move):
    state_msg = ModelState()

    state_msg.model_name = objeto
    state_msg.pose.position.x = goal_x
    state_msg.pose.position.y = goal_y
    state_msg.pose.position.z = goal_z
    state_msg.pose.orientation.x = 1
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
       set_state = rospy.ServiceProxy(
          '/gazebo/set_model_state', SetModelState)
       resp = set_state(state_msg)
       print(state_msg)

    except rospy.ServiceException, e:
       print "Service call failed: %s" % e

def get_robot_position():
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates('tiago', '')
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    if VERBOSE:
        print 'Status.success = ', resp_coordinates.success
        print("robot pose " + str(resp_coordinates.pose.position.x))
    return resp_coordinates.pose.position.x, resp_coordinates.pose.position.y


def get_closest_table_position():
    min_distance = 1000000
    closest_table_position = np.array(0,0)
    for table in self.list_of_tables:
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(table, '')
            curr_table_coords = np.array(resp_coordinates.pose.position.x,
                resp_coordinates.pose.position.y)
            if (get_robot_position() - curr_table_coords).norm()  < min_distance:
                closest_table_position = curr_table_coords
        except rospy.ServiceException, e:
            print "ServiceProxy failed: %s"%e
            #exit(0)
    
    return closest_table_position




def get_closest_table_position_old():
    try:
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    model_prop = get_model_properties("door_simple")
    try:
        get_door_joint_props = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    if VERBOSE: print('---------- door aperture ---------')
    joint_prop = get_door_joint_props('joint_frame_door')
    if VERBOSE: print(joint_prop.position[0])
   
    return joint_prop.position[0]

def listener(self):
#    rospy.init_node('eurobench_worldstate_provider', anonymous=True)
    image_camera = rospy.Subscriber("sensor_msgs/Image", Image, callback)
    

def getTrolleyPosition(): 
    try:
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    model_prop = get_model_properties("door_simple")
    try:
        get_door_joint_props = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    if VERBOSE: print('---------- door aperture ---------')
    joint_prop = get_door_joint_props('joint_frame_door')
    if VERBOSE: print(joint_prop.position[0])
   
    return joint_prop.position[0]


def getHandlePosition():
    try:
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    model_prop = get_model_properties("pushcart")
    try:
        get_door_joint_props = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    joint_prop_handle = get_door_joint_props('joint_door_lever')
    if VERBOSE: 
        print('---------- handle position ---------')
    if VERBOSE: 
        print(joint_prop_handle.position[0])
        
    getTrolleyPosition()
        
    return joint_prop_handle.position[0]
    
    
    

def getTrolleyPosition():
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates('pushcart::cart_front_steer', 'chassis')
        print '\n'
        print 'Status.success = ', resp_coordinates.success
        print("---- Pushcart pose \n: " + str(resp_coordinates.pose.position))
        #print("Quaternion of X : " + str(resp_coordinates.pose.orientation.x))

    except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))
        
    #return joint_prop_handle.position[0]


def retrieveBenchmarkConfiguration(ebws):    # Based on the currently selected benchmark type
    try:
        get_benchmark_params = rospy.ServiceProxy('beast/gui/benchmark_params', BeastBenchmarkParams)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    response = get_benchmark_params()
    ebws.stiffness = response.stiffness
    if VERBOSE:
        print "\n---- trolley stiffness from gui:"
        print ebws.stiffness


def benchmarkConfigurationHasChanged(ebws):
    if (ebws.current_door_opening_side != ebws.old_door_opening_side
    or ebws.current_robot_approach_side != ebws.old_robot_approach_side):
        # to restart also when simulation force change use 
        # ebws.current_benchmark_name != ebws.old_benchmark_name
        ebws.old_benchmark_name = ebws.current_benchmark_name
        ebws.old_door_opening_side = ebws.current_door_opening_side
        ebws.old_robot_approach_side = ebws.current_robot_approach_side
        if VERBOSE: 
            print("PARAMETERS ARE CHANGED")
            
        return True;
    return False;
        

#def getScene(benchmark_name):
#    scene_map = {
#        "No Force": noforce,
#        "Constant Force": constant_force,
#        "Sudden Force": sudden_force,
#        "Sudden Ramp": sudden_ramp,
#        "Wind Ramp": wind_ramp,
 #   }
#    func = scene_map.get(benchmark_name)
#    arg1, arg2 = func()
#    return arg1, arg2
    


def main(args):
     ebws =  sciroc_ep1_object_manager()
     rospy.init_node('sciroc_ep1_object_manager', anonymous=True) #CHECK IF REMOVE 'PROVIDER'
     
     listener(ebws)

     #s = rospy.Service('/beast/trolley/set_stiffness', SetStiffness, handle_beast_trolley_dummy_srv) 
     s = rospy.Service('/sciroc_object_manager/reset_tray', ResetTray, reset_tray_srv) 
     s = rospy.Service('/sciroc_object_manager/move_objects_on_the_closest_table', MoveObjectsOnClosesTable, move_objects_on_the_closest_table_srv) 
     #print ("service reset_tray and move_objects_on_the_closest_table in sciroc_ep1_object_manager_node")    

     try:
         talker(ebws)
         rospy.spin()
     except KeyboardInterrupt:
           print ("Shutting down sciroc_ep1_object_manager module")



if __name__ == '__main__':
     main(sys.argv)
