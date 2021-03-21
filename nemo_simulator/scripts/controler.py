#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Starts a new node
rospy.init_node('controler', anonymous=True)
rate = rospy.Rate(10)  # 10Hz


class Robot():
    def __init__(self):
        # declare the variables
        self.dx = 0  # sonar distance x, y and z
        self.dy = 0
        self.dz = 0
        self.opx = 0  # odometer position x, y and z
        self.opy = 0
        self.opz = 0
        self.oox = 0  # odometer orientation x, y and z
        self.ooy = 0
        self.ooz = 0
        self.vx = 0  # robot's speed vectors
        self.vy = 0
        self.vz = 0
        self.az = 0  # angular speed around z axis
        self.targetdx = 0
        self.targetdy = 0
        self.targetdz = 0
        self.distance = 0

        # create the publisher and subscribers for each sensor
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/sonar_data', PointStamped, self.sonar_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('target_pos', Pose, self.pos_callback)

    def sonar_callback(self, sonar):
        self.dx = sonar.point.x
        self.dy = sonar.point.y
        self.dz = sonar.point.z

    def odom_callback(self, odom):
        self.opx = odom.pose.pose.position.x
        self.opy = odom.pose.pose.position.y
        self.opz = odom.pose.pose.position.z

        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        orientation_list = (x, y, z, w)

        (self.oox, self.ooy, self.ooz) = euler_from_quaternion(orientation_list)

    def pos_callback(self, pos):
        self.targetdx = pos.position.x
        self.targetdy = pos.position.y
        self.targetdz = pos.position.z

#########################################################################################################################
############# MOVE FUNCTION #############################################################################################
#########################################################################################################################


def move():
    vel_msg = Twist()

    # linear and angular velocity vectors
    vel_msg.linear.x = marlin.vx
    vel_msg.linear.y = marlin.vy
    vel_msg.linear.z = marlin.vz
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = marlin.az

    marlin.vel_pub.publish(vel_msg)


#########################################################################################################################
############# SONAR FUNCTION ############################################################################################
#########################################################################################################################

def sonar():

    # using the data from the sonar, we calculate the distance between the two robots
    marlin.distance = math.sqrt(((marlin.dx)**2) + ((marlin.dy)**2))
    #print marlin.distance

    # print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    # print "delta-X:", marlin.dx
    # print "delta-Y:", marlin.dy
    # print "delta-Z:", marlin.dz
    # print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    # print


#########################################################################################################################
############# ODOMETER FUNCTION #########################################################################################
#########################################################################################################################

def odom():
    print "#############################"
    print "odom-X:", marlin.opx
    print "odom-Y:", marlin.opy
    print "odom-Z:", marlin.opz
    #print "odomAng-X:", marlin.oox
    #print "odomAng-Y:", marlin.ooy
    print "odomAng-Z:", marlin.ooz
    print "#############################"
    print


#########################################################################################################################
############# SEARCH FUNCTION ###########################################################################################
#########################################################################################################################

#       C(x:-10, y: -10)----------------(x:-10, y: 10)B
#       |                                             |
#       |                                             |
#       |                                             |
#       |                                             |
#       |                                             |
#       |      COORDINATES OF THE MAP'S CORNERS       |
#       |                                             |
#       |                                 start pos (x: 5, y: 7)
#       |                                             |
#       |                                             |
#       |                                             |
#       D(x: 10, y: -10)----------------(x: 10, y: 10)A

def search():
    marlin.vx = 0
    marlin.vy = 4

    # brick wall avoiding algorithm
    if -3.5 < marlin.opx < 3.5 and 2 < marlin.opy < 3.2:  # if we enter the region near the brick wall
        if(-math.pi < marlin.ooz < -math.pi/2):
            # get the speed, decompose it and make it go "upwards", not hitting the wall
            prevvy = marlin.vy
            marlin.vy = (prevvy*math.sin((math.pi/2)+(math.pi-marlin.ooz)))/(0.1 + math.sin(marlin.ooz)*(math.sin((math.pi/2)+(math.pi-marlin.ooz))-1))
            marlin.vx = 0
            marlin.vx = -1*marlin.vy*math.cos((math.pi) + marlin.ooz)/math.cos((math.pi/2)+(math.pi-marlin.ooz))

        if(math.pi/2 < marlin.ooz < math.pi):
            # get the speed, decompose it and make it go "upwards", not hitting the wall
            prevvy = marlin.vy
            marlin.vy = (prevvy*math.sin((math.pi/2)-marlin.ooz))/(0.1 + math.sin(marlin.ooz)*(math.sin((math.pi/2)-marlin.ooz)-1))
            marlin.vx = 0
            marlin.vx = -1*marlin.vy*math.cos(marlin.ooz)/math.cos((math.pi/2)-marlin.ooz)
    
    if -3.5 < marlin.opx < 3.5 and -3.2 < marlin.opy < -2:  # if we enter the region near the brick wall
        if(-math.pi/2 < marlin.ooz < 0):
            # get the speed, decompose it and make it go "upwards", not hitting the wall
            prevvy = marlin.vy
            marlin.vy = (prevvy*math.sin((math.pi/2)-marlin.ooz))/(0.1 + math.sin(marlin.ooz)*(math.sin((math.pi/2)-marlin.ooz)-1))
            marlin.vx = 0
            marlin.vx = -1*marlin.vy*math.cos(marlin.ooz)/math.cos((math.pi/2)-marlin.ooz)

        if(0 < marlin.ooz < math.pi/2):
            # get the speed, decompose it and make it go "upwards", not hitting the wall
            prevvy = marlin.vy
            marlin.vy = (prevvy*math.sin((math.pi/2)+(math.pi-marlin.ooz)))/(0.1 + math.sin(marlin.ooz)*(math.sin((math.pi/2)+(math.pi-marlin.ooz))-1))
            marlin.vx = 0
            marlin.vx = -1*marlin.vy*math.cos((math.pi) + marlin.ooz)/math.cos((math.pi/2)+(math.pi-marlin.ooz))

    # move around the perimeter of the map
    if marlin.opx > -10 and marlin.opy > 10:  # rect region between B and A
        marlin.vx = 0
        marlin.vy = 0

        # rotate 180 degrees
        target = -3.14
        marlin.az = 6 * (target - marlin.ooz)
        if(marlin.ooz < target + 0.04 and marlin.ooz > target - 0.04):
            marlin.vx = 4

    elif marlin.opx < -10 and marlin.opy > -10:  # rect region between C and B
        marlin.vx = 0
        marlin.vy = 0

        # rotate 90 degrees
        target = -1.57
        marlin.az = 8 * (target - marlin.ooz)
        if(marlin.ooz < target + 0.04 and marlin.ooz > target - 0.04):
            marlin.vx = 4

    if marlin.opx < 10 and marlin.opy < -10:  # rect region between D and C
        marlin.vx = 0
        marlin.vy = 0

        # rotate 90 degrees
        target = 0
        marlin.az = 8 * (target - marlin.ooz)
        if(marlin.ooz < target + 0.04 and marlin.ooz > target - 0.04):
            marlin.vx = 4

    if marlin.opx > 10 and marlin.opy < 10:  # rect region between A and D
        marlin.vx = 0
        marlin.vy = 0

        # rotate 90 degrees
        target = 1.57
        marlin.az = 8 * (target - marlin.ooz)
        if(marlin.ooz < target + 0.04 and marlin.ooz > target - 0.04):
            marlin.vx = 4


#########################################################################################################################
############# FOLLOW FUNCTION ###########################################################################################
#########################################################################################################################

def follow():
    # drive and rotate towards the cylinder
    marlin.vx = 0
    marlin.vy = 4 * -20/(marlin.targetdy + 900) # marlin.targetdy is related to the distance. So the grater the distance, the greater the speed
    marlin.az = -8 * (marlin.targetdx)/1000

    # brick wall avoiding algorithm
    if -4.8 < marlin.opx < 4.8 and -3.2 < marlin.opy < 3.2:  # if we enter the region near the brick wall
        if(0 < marlin.ooz < math.pi/2): # y
            # get the speed, decompose it and make it go "upwards", not hitting the wall
            prevvy = marlin.vy
            marlin.vy = (prevvy*math.sin((math.pi/2)-marlin.ooz))/(0.1 + math.sin(marlin.ooz)*(math.sin((math.pi/2)-marlin.ooz)-1))
            marlin.vx = 0
            marlin.vx = 1*marlin.vy*math.cos(marlin.ooz)/math.cos((math.pi/2)-marlin.ooz)
        if(math.pi/2 < marlin.ooz < math.pi): # y
            # get the speed, decompose it and make it go "upwards", not hitting the wall
            prevvy = marlin.vy
            marlin.vy = (prevvy*math.sin((math.pi/2)-marlin.ooz))/(0.1 + math.sin(marlin.ooz)*(math.sin((math.pi/2)-marlin.ooz)-1))
            marlin.vx = 0
            marlin.vx = -1*marlin.vy*math.cos(marlin.ooz)/math.cos((math.pi/2)-marlin.ooz)
        if(-math.pi < marlin.ooz < -math.pi/2): # x
            # get the speed, decompose it and make it go "upwards", not hitting the wall
            prevvy = marlin.vy
            marlin.vx = (prevvy*math.sin((math.pi/2)-marlin.ooz))/(0.1 + math.sin(marlin.ooz)*(math.sin((math.pi/2)-marlin.ooz)-1))
            marlin.vy = 0
            marlin.vy = 1*marlin.vx*math.cos(marlin.ooz)/math.cos((math.pi/2)-marlin.ooz)
        if(-math.pi/2 < marlin.ooz < 0): # x
            # get the speed, decompose it and make it go "upwards", not hitting the wall
            prevvy = marlin.vy
            marlin.vx = (prevvy*math.sin((math.pi/2)-marlin.ooz))/(0.1 + math.sin(marlin.ooz)*(math.sin((math.pi/2)-marlin.ooz)-1))
            marlin.vy = 0
            marlin.vy = -1*marlin.vx*math.cos(marlin.ooz)/math.cos((math.pi/2)-marlin.ooz)

    # if the cylinder is centered, we go forward
    if marlin.targetdx < 100 and marlin.targetdx > -100:
        marlin.az = 0
        marlin.vx = 0
        marlin.vy = 3.5
        #brick wall avoiding algorithm
        if -4.8 < marlin.opx < 4.8 and -3.2 < marlin.opy < 3.2:  # if we enter the region near the brick wall
            # get the speed, decompose it and make it go "upwards", not hitting the wall
            prevvy = marlin.vy
            marlin.vy = 0
            marlin.vx = 0
            marlin.vy = prevvy*math.sin(180*marlin.ooz/math.pi)
            marlin.vx = -1*prevvy*math.cos(180*marlin.ooz/math.pi)
    
    # if we are too close, we stop
    if marlin.distance < 3:
        marlin.vy = 0
        marlin.vx = 0

def shutdownhook():
            # works better than the rospy.is_shut_down()
            global ctrl_c
            print "stopping the robot"
            marlin.vx = 0
            marlin.vy = 0
            marlin.az = 0
            move()
            ctrl_c = True

rospy.on_shutdown(shutdownhook)


marlin = Robot()
    
while not rospy.is_shutdown():
    #print marlin.targetdy
    if marlin.targetdx == 0:
        search()
    else:
        follow()
    move()
    sonar()
    odom()
    rate.sleep()

rospy.spin()
