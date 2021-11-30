#!/usr/bin/env python
 

from pickle import STRING
import re
import string
import math
from types import NoneType
from numpy.lib.function_base import append
import time
from numpy.linalg.linalg import qr
import rospy
import actionlib
from geometry_msgs.msg import Twist, PoseStamped
import tf
import re
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import dynamic_reconfigure.client
#from actionlib_msgs import GoalID
import move_base



targets= [] #list for targets
position = None
#data class for the QR codes
class qrData:
    def __init__(self, posX, posY, nextX, nextY, id, letter, position):
        self.posX = posX
        self.posY = posY
        self.nextX = nextX
        self.nextY = nextY
        self.id = id
        self.letter = letter
        self.visited = False
        self.position = position

 
def robotParameters():
    client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')
    try:
        new_params = {
            'min_vel_x' : -0.16,
            'min_vel_theta' : 0.1,
            'max_vel_x' : 0.16,
            'max_vel_theta' : 0.2
        }
        config = client.update_configuration(new_params)
        return True
    except rospy.ServiceException as e:
        print("Fail: %s", e)
        return False

def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
 
    return goal_pose



def callbackPOS(data):
    if data.pose.position.x != 0.0 :
        data.header.frame_id = 'camera_optical_link'
        #print("Pre:", data)
        data = goaler.transformPose('/map',data)
        global position
        position = data
        #print(position)


#get data from QR codes and put it into the targets list (sorted by id)
def callbackMSG(data):
    global targets
    Data = re.findall(r"[-+]?\d*\.\d+|\d+", str(data))
    
    if len(Data):
        if not len(targets):
            #client.cancel_all_goals()
            rospy.sleep(5)
            qrd= qrData(float(Data[0]),float(Data[1]),float(Data[2]),float(Data[3]),int(Data[4]),str(Data[-1]),position)
            targets.append(qrd)

            
            
        else:
            
            qrd= qrData(float(Data[0]),float(Data[1]),float(Data[2]),float(Data[3]),int(Data[4]),str(Data[-1]),position)
            found = False
            for dat in targets:
                if dat.id == qrd.id:
                    found = True
            if not found:

                targets.append(qrd)
            targets.sort(key= lambda x: x.id) 

    else:
        return

def rotate(move):
    rotate = Twist()
    
    if move:
        rotate.angular.z = 0.1
    else:
        rotate.angular.z = 0
    return rotate

start_time= time.time()
seconds = 60


def rotateRobot():
    #Starts a new node
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    speed = 2
    angle = 360
    clockwise = True

    #Converting from angles to radians
    angular_speed = speed*2*math.pi/360
    relative_angle = angle*2*math.pi/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
        print(current_angle,relative_angle)

    print("end")
    #Forcing our robot to stop
    vel_msg.angular.z = 0
    print("here1")
    velocity_publisher.publish(vel_msg)
    return


def findFirst():
    global targets
    if not len(targets):
        goal = goal_pose([(-4.0,0.0, 0.0), (0.0,0.0, 0.0, 0.5)])
        client.send_goal(goal)
        client.wait_for_result()
        rospy.sleep(3)
        targets = []
        rotateRobot()
    print("here4")
    if not len(targets):
        goal = goal_pose([(5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.5)])
        client.send_goal(goal)
        client.wait_for_result()
        rospy.sleep(3)
        targets = []
        rotateRobot()
    #if not len(targets):
        #rotateRobot()tf.
    #print(targets[0].pos)

def goToQR():
    global targets
    target = targets[1]
    print('Next target:',target.position.pose.position.x,target.position.pose.position.y)
    goal = goal_pose([(target.position.pose.position.x,target.position.pose.position.y, 0.0), (0.0, 0.0, 0.0,0.5)])
    client.send_goal (goal)
    client.wait_for_result()


def goToFirst():
    print("KKKKKKKKKKKKKKKKKKKKKKKKK")
    target= targets[0]
    print('First target: ',target.id, target.position.pose.position.x,target.position.pose.position.y )
    goal = goal_pose([(target.position.pose.position.x,target.position.pose.position.y, 0.0), (0.0, 0.0, 0.0,0.5)])
    client.send_goal (goal)
    client.wait_for_result()
    if client.get_result():
        target.visited = True
    rospy.sleep(3)
    #goToQR(target[1])

def getTargIdx(id):
    for target in targets:
        if target.id == id:
            return targets.index(target)


if __name__ == '__main__':
    rospy.init_node('patrol')
    subMSG = rospy.Subscriber('/visp_auto_tracker/code_message', String , callback=callbackMSG)
    subPOS = rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, callback= callbackPOS )
    goaler = tf.TransformListener()
    robotParameters()
    #cancel_pub = rospy.Publisher("/move_base/cancel", GoalID,queue_size=1)
    rate = rospy.Rate(10)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    findFirst()
    goToFirst()
    goToQR()
    
    """
    while True:
        for pose in waypoints:   
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
            rospy.sleep(3)
    """
