#!/usr/bin/env python

import math
import rospy
import roslaunch
import rospkg
import os
import sys
import tf
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float64
from sciroc_ep1_object_manager.srv import MoveItemsOnTheTray
from sciroc_ep1_object_manager.srv import MoveItemsOnClosestTable
from sciroc_ep1_object_manager.srv import GetThreeOrderedItems
from sciroc_ep1_object_manager.srv import ChangeTheItem
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelProperties
from gazebo_msgs.srv import GetJointProperties

from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel


VERBOSE = True

object_counter = 0

ROBOT_TRAY_HEIGHT = 1.2             #TODO
TABLE_CAFFE_HEIGHT = 1              #TODO
COUNTER_H = 1.3                     #TODO
COUNTER_POSE = np.array([4.5, -1.4, COUNTER_H])
MIN_DIST_TO_MOVE_OBJS = 1.5         #TODO
# distance of objects from the center of the table 
OFFSET = 0.2
OFFSET_TRAY = 0.1

SPAWN_POSE_1 = Pose(position=Point(x=4.5, y=-1.4+OFFSET, z=COUNTER_H))
SPAWN_POSE_2 = Pose(position=Point(x=4.5, y=-1.4, z=COUNTER_H))
SPAWN_POSE_3 = Pose(position=Point(x=4.5, y=-1.4-OFFSET, z=COUNTER_H))


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
        
        self.objects_on_robot_tray = {
            "none",
            "none",
            "none"
        }

        self.counter_object = {"table"}				

          
    def startSim(self):
        package = 'eurobench_reemc_cart'
        launch_file = 'reemc_cart.launch'
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
     #   sys.argv = [ 'door:=simple', 'direction:=push', 'robot_placement_cw:=true']
        
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self.launch.start()

 
    def cw_left_callback(self, ros_data):
        self.publish_strips(self.cw_left, ros_data, self.cw_left_pub)

    def cw_right_callback(self, ros_data):
        self.publish_strips(self.cw_right, ros_data, self.cw_right_pub)
                

    


def talker(se1om):
    if VERBOSE:
        print ("TALKER")

    r = rospy.Rate(10) #10hz

    msg = Float64()
#    load_and_spawn_gazebo_models("beer", 4.5, -2, 1.6)


    spawn_three_objs("beer", "beer", "beer", se1om)
    move_items_on_the_tray()
        
    
    
    while not rospy.is_shutdown():
        #msg = getDoorAperture()
        #ebws.door_pub.publish(msg)
        
        #get_robot_position()
        #get_robot_orientation()
        #load_and_spawn_gazebo_models("beer", 4.5, -2, 1.6)
        #spawn_three_objs("beer", "beer", "beer")
        print(get_closest_table_position_and_distance(se1om))
        get_robot_tray_position()
        
        print("BEER SPAWNED")
        #msg_handle = getTrolleyPosition()
        #ebws.door_handle_pub.publish(msg_handle)


        
        r.sleep()


def callback(data):
    rospy.loginfo("%s is age: %d" % (data.name, data.age))
    print ("initialized")


def move_items_on_the_tray():   #TO REMOVE!!!!!!!!!
    print("move_objects_on_the_tray_srv service")
    
    counter_distance = get_robot_counter_distance()
    if counter_distance > MIN_DIST_TO_MOVE_OBJS:
        return 
 
    tray_pose = get_robot_tray_position()
#    self.objects_on_robot_tray #TODO check if needed to be put on global var 
    set_position(tray_pose[0], 
                tray_pose[1],
                tray_pose[2], 
                self.objects_on_robot_tray[0])

    set_position(tray_pose[0], 
                tray_pose[1],
                tray_pose[2], 
                self.objects_on_robot_tray[1])

    set_position(tray_pose[0], 
                tray_pose[1],
                tray_pose[2], 
                self.objects_on_robot_tray[2])
    print("move_objects_on_the_tray_srv service")
    return MoveItemsOnTheTray.srvResponse(True, "")



def move_items_on_the_tray_srv(req):  
    print("move_objects_on_the_tray_srv service")
    
    counter_distance = get_robot_counter_distance()
    if counter_distance > MIN_DIST_TO_MOVE_OBJS:
        return MoveItemsOnClosestTable.srvResponse(False, "")
 
    tray_pose = get_robot_tray_position()
#    self.objects_on_robot_tray #TODO check if needed to be put on global var 
    set_position(tray_pose[0], 
                tray_pose[1],
                tray_pose[2], 
                self.objects_on_robot_tray[0])

    set_position(tray_pose[0], 
                tray_pose[1],
                tray_pose[2], 
                self.objects_on_robot_tray[1])

    set_position(tray_pose[0], 
                tray_pose[1],
                tray_pose[2], 
                self.objects_on_robot_tray[2])
    print("move_objects_on_the_tray_srv service")
    return MoveItemsOnTheTray.srvResponse(True, "")

def move_items_on_the_closest_table_srv(req):  
    #TODO
    closest_table_position, table_distance = get_closest_table_position_and_distance()
    if table_distance > MIN_DIST_TO_MOVE_OBJS:
        return MoveItemsOnTheTray.srvResponse(False, "")
    
    
    
#    self.objects_on_robot_tray #TODO check if needed to be put on global var 
    set_position(closest_table_position.x - OFFSET, 
                closest_table_position.y + OFFSET,
                TABLE_CAFFE_HEIGHT, 
                self.objects_on_robot_tray[0])

    set_position(closest_table_position.x + OFFSET, 
                closest_table_position.y - OFFSET,
                TABLE_CAFFE_HEIGHT, 
                self.objects_on_robot_tray[1])

    set_position(closest_table_position.x + OFFSET, 
                closest_table_position.y + OFFSET,
                TABLE_CAFFE_HEIGHT, 
                self.objects_on_robot_tray[2])
    print("move_objects_on_the_closest_table_srv service")
    return MoveObjectsOnClosestTable.srvResponse(True, "")
    
def get_three_ordered_items_srv(req):  
    #TODO
    # string1, string2, string3
    
    
    print("get_three_objects_srv service")
    return GetThreeOrderedItems.srvResponse(True, "")
    
def spawn_three_objs(obj0, obj1, obj2, monitor):
    #TODO AGGINUGERE ERRORE 1 su 3
    global TABLE_BANK_POSE
    global OFFSET
    modlist, model0 = load_and_spawn_gazebo_models(obj0, SPAWN_POSE_1)   
    modlist, model1 = load_and_spawn_gazebo_models(obj1, SPAWN_POSE_2)   
    modlist, model2 = load_and_spawn_gazebo_models(obj2, SPAWN_POSE_3)   
    monitor.objects_on_robot_tray = {model0, model1, model2}

    
def change_the_item_srv(req):  
    #TODO cambiare modelli di objects_on_robot_tray
    req.name_of_the_object_to_change
    print("change_the_objects_srv service")
    return ChangeTheItem.srvResponse(True, "")
    
  
def spawn_sdf(name, description_xml, pose, reference_frame):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf(name, description_xml, "/", pose, reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e)) 
  
def spawn_sdf_model(name, path, pose, reference_frame):
    # Load Model SDF
    description_xml = ''
    with open(path, "r") as model_file:
        description_xml = model_file.read().replace('\n', '')
        spawn_sdf(name, description_xml, pose,reference_frame)
        
def load_and_spawn_gazebo_models(obj_name, pose):   
    model_list = []
    world_reference_frame = "world"
    # sorting_demo model path  
    ep1_models_path = rospkg.RosPack().get_path('sciroc_ep1_object_manager') + "/models/" 
    # Spawn object
    object_name = obj_name
    object_path = ep1_models_path + "/" + object_name+ "/model.sdf"
    #object_pose = Pose(position=Point(x=x_obj, y=y_obj, z=z_obj))
    object_pose = pose
    global object_counter 
    spawn_sdf_model(object_name+str(object_counter), 
        object_path, 
        object_pose, 
        world_reference_frame)
    current_model = object_name+str(object_counter)
    model_list.append(current_model)
    object_counter+= 1
    # Spawn Trays Table
    #trays_table_name = "trays_table"
    #trays_table_path = sorting_demo_models_path + "table/model.sdf"
    #trays_table_pose = Pose(position=Point(x=0.0, y=0.95, z=0.0))

    #spawn_sdf_model(trays_table_name, trays_table_path, trays_table_pose, world_reference_frame)
    #model_list.append(trays_table_name)
    return model_list, current_model
    
def is_there_an_object_on(x,y,z):
    return False

def get_scene_object_list(req):
    return []


def set_position(goal_x, goal_y,goal_z, object_to_move):
    state_msg = ModelState()

    state_msg.model_name = object_to_move
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

def get_robot_position():           #OK
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates('tiago', '')
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    if VERBOSE:
        print 'Status.success = ', resp_coordinates.success
        print("robot pose " + str(resp_coordinates.pose.position.x))
        print("robot pose " + str(resp_coordinates.pose.position.y))
    return np.array([resp_coordinates.pose.position.x, resp_coordinates.pose.position.y])
    
def get_robot_orientation():        #OK
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates('tiago', '')
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    quaternion = (
        resp_coordinates.pose.orientation.x,
        resp_coordinates.pose.orientation.y,
        resp_coordinates.pose.orientation.z,
        resp_coordinates.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    if VERBOSE:
        print 'Status.success = ', resp_coordinates.success
        print("robot orientation " + str(yaw))
    return yaw
    
def get_robot_tray_position():          
    robot_pose = get_robot_position()
    robot_orientation = get_robot_orientation()
    
    return np.array(
        [robot_pose[0] + OFFSET_TRAY*math.cos(robot_orientation), 
        robot_pose[1] + OFFSET_TRAY*math.sin(robot_orientation),
        ROBOT_TRAY_HEIGHT]
    )
    


def get_closest_table_position_and_distance(se1om): #OK
    min_distance = 1000000
    closest_table_position = np.array([0,0])
    for table in se1om.list_of_tables:
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(table, '')
            curr_table_coords = np.array([
                resp_coordinates.pose.position.x,
                resp_coordinates.pose.position.y])
#            curr_min_dist = (get_robot_position() - curr_table_coords).norm()
            curr_min_dist = np.linalg.norm(get_robot_position() - curr_table_coords)
            if curr_min_dist  < min_distance:
                closest_table_position = curr_table_coords
                min_distance = curr_min_dist
        except rospy.ServiceException, e:
            print "ServiceProxy failed: %s"%e
            #exit(0)
    
    return closest_table_position, min_distance


def get_robot_counter_distance(se1om): #OK
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(se1om.counter_object, '')
        counter_coords = np.array([
            resp_coordinates.pose.position.x,
            resp_coordinates.pose.position.y])
#            curr_min_dist = (get_robot_position() - curr_table_coords).norm()
        dist = np.linalg.norm(get_robot_position() - counter_coords)
        
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        #exit(0)
    
    return dist



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
     se1om =  sciroc_ep1_object_manager()
     rospy.init_node('sciroc_ep1_object_manager', anonymous=True) #CHECK IF REMOVE 'PROVIDER'
     
     listener(se1om)

     #s = rospy.Service('/beast/trolley/set_stiffness', SetStiffness, handle_beast_trolley_dummy_srv) 
     s = rospy.Service('/sciroc_object_manager/move_items_on_the_tray', MoveItemsOnTheTray, move_items_on_the_tray_srv) 
     s = rospy.Service('/sciroc_object_manager/move_items_on_the_closest_table', MoveItemsOnClosestTable, move_items_on_the_closest_table_srv) 
     s = rospy.Service('/sciroc_object_manager/get_three_ordered_items', GetThreeOrderedItems, get_three_ordered_items_srv) 
     s = rospy.Service('/sciroc_object_manager/change_the_item', ChangeTheItem, change_the_item_srv) 
     
     #print ("service reset_tray and move_objects_on_the_closest_table in sciroc_ep1_object_manager_node")    

     try:
         talker(se1om)
         rospy.spin()
     except KeyboardInterrupt:
           print ("Shutting down sciroc_ep1_object_manager module")



if __name__ == '__main__':
     main(sys.argv)
