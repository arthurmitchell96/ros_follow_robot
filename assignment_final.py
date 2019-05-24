# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import Image
#from sensor_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv2 import namedWindow, cvtColor, imshow
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from kobuki_msgs.msg import BumperEvent


#initilises first pass bool 		
global FIRST
FIRST = True
print FIRST

class Follower:
	laser = []
	dist = []
	def __init__(self):
		#creates bride to dislpay image from robot
		self.bridge = cv_bridge.CvBridge()
		#opens image window
		cv2.namedWindow("image window", 1)
		cv2.startWindowThread()
		#subscribes to laser scan
		self.depth_sub = rospy.Subscriber('/turtlebot/scan', LaserScan, self.depth_callback)
		#subscribes to laserscan
		self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw',Image, self.image_callback)
		#subscribes to movement controlls
		self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		
		#start moving	
		self.twist = Twist()
	
	def depth_callback(self, msg):
		#declares global variable wall to stop movemnt controlls when near wall
		global WALL
		#make global variable for laser scanner data	
		self.laser = msg
				
		#finds minum distance from laserscan ranges
		self.dist = min(msg.ranges)
		#cycles through ranges to check how close the nearest object is
		for range in msg.ranges:
			self.dist = range
			#if nearest object is less than 1.25m away stop and trun left		
			if self.dist <1.25:
				#triggers wall variable to stop other movement controlls
				WALL = True
				self.twist.linear.x = 0
			  	#hard turn
				self.twist.angular.z= 5
				#publish movement commands
				self.cmd_vel_pub.publish(self.twist)
				#print 'hit'
			#turn harder if theres more space to find wall
			elif self.dist > 3:
				self.twist.angular.z = 0.2
				self.cmd_vel_pub.publish(self.twist)
			elif self.dist == 0 :
				self.twist.linear.x = 0
				self.cmd_vel_pub.publish(self.twist)
			else:
				#if distance is less over 1.25m allow other movement controlls to work
				WALL = False
				
				#print 'wall false'
			
		#global max range varibale			
		self.maxRange = msg.range_max		

		
		
	#function to find colours
	def colour_thresh(self, image2scan):
		#if found colour set flag to true and remove that colour from the filter
		if (image2scan[self.h/2,self.w/2][0]>=0 and image2scan[self.h/2,self.w/2][0] <=20):
			self.red_found = True
			
			print image2scan[self.h/2,self.w/2]
			print 'found red'
			#turn to give a better chance of finding the other flags
			self.twist.angular.z = 2
			self.cmd_vel_pub.publish(self.twist)
			time.sleep(5)
			
			return  'red'
		elif (image2scan[self.h/2,self.w/2][0]>=30 and image2scan[self.h/2,self.w/2][0] <=50):
			self.yellow_found = True
			
			print image2scan[self.h/2,self.w/2]
			print 'found yellow'
			self.twist.angular.z = 2
			self.cmd_vel_pub.publish(self.twist)
			time.sleep(5)
			return  'yellow'
		elif (image2scan[self.h/2,self.w/2][0]>=60 and image2scan[self.h/2,self.w/2][0] <=75):
			self.green_found = True
			
			print image2scan[self.h/2,self.w/2]
			print 'found green'
			self.twist.angular.z = 2
			self.cmd_vel_pub.publish(self.twist)
			time.sleep(5)
			return  'green'
		elif (image2scan[self.h/2,self.w/2][0]>=100 and image2scan[self.h/2,self.w/2][0] <=130):
			self.blue_found = True
			
			print image2scan[self.h/2,self.w/2]
			print 'found blue'
			self.twist.angular.z = 2
			self.cmd_vel_pub.publish(self.twist)
			time.sleep(5)
			return  'blue'
	def furthest_angle(self):
		angle = self.dist.index(max(self.dist))
		return angle
		
	def image_callback(self, msg):
		global FIRST
		#all found to declare when when all colours found
		all_found= False
		global WALL
		#set the filters for each colour
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		#gets raw camera image in hsv
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		#yellow
		self.lower_yellow = numpy.array([30, 50, 50])
		self.upper_yellow = numpy.array([60, 255, 250])
		#red
		self.lower_red = numpy.array([0, 100, 100])
		self.upper_red = numpy.array([10, 255, 255])
		#green
		self.lower_green = numpy.array([60, 100, 100])
		self.upper_green = numpy.array([75, 255, 255])
		#blue
		self.lower_blue = numpy.array([100, 150, 50])
		self.upper_blue = numpy.array([130, 255, 255])
		#set mask to zero on each pass
		self.set_lower = numpy.array([0, 0, 0])
		#create mask
		self.mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
		self.mask_red = cv2.inRange(hsv, self.lower_red, self.upper_red)
		self.mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
		self.mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
		if FIRST:
			#set flags to false only on first pass
			print'mask set'
			self.red_found = False
			self.yellow_found = False
			self.green_found = False
			self.blue_found = False
			
			FIRST=False
		#set mask to empy on each pass	
		self.mask = cv2.inRange(hsv, self.set_lower, self.set_lower)
		#if that colour hansn't been found add to mask 
		if (self.yellow_found == False):
			self.mask+= self.mask_yellow
		if (self.red_found == False):
			self.mask+= self.mask_red
		if (self.blue_found == False):
			self.mask+= self.mask_blue
		if (self.green_found == False):
			self.mask+= self.mask_green
		#create mask
		res = cv2.bitwise_and(hsv,hsv, mask= (self.mask)) 
		#if all found shutdown
		if self.blue_found and self.red_found and self.yellow_found and self.green_found:
			print 'all found'
			all_found = True
			rospy.on_shutdown(self.shutdown)
		#set search area for objects within mask	
		self.h, self.w, self.d = image.shape
		search_top = 3*self.h/4 - 100
		search_bot = 3*self.h/4 + 100
		self.mask[0:search_top, 0:self.w-(self.w-100)] = 0
		self.mask[search_bot:self.h, 0:self.w-(self.w-400)] = 0
		M = cv2.moments(self.mask)
		#if object size is over a certian size(to get rid of false postives) and that theres isnt walls 
		
		if M['m00'] > 15000  and WALL == False  :
			#print M['m00']
			#gets x and y of target 
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(res, ((cx), (cy-150)), 20, (0,0,255), -1)

			if self.dist <1.25:
				WALL = True
			#workout how far away form target robot is 
			err = cx - self.w/2
			#set linear vel 
			self.twist.linear.x = 0.5
			self.twist.angular.z = -float(err)/100
			self.cmd_vel_pub.publish(self.twist)
			#once object is big enough stop and check what colour it is 
			if M['m00'] > 4000000:
				self.twist.linear.x = 0.0
				self.cmd_vel_pub.publish(self.twist)
				found_colour = self.colour_thresh(hsv) 
				print image[self.h/2,self.w/2]
				print found_colour
				
				 
		#else rive forward and to the right to find the wall to follow	
		elif WALL == False:
			#print 'not trigger'
			
			self.twist.linear.x= 0.2
			self.twist.angular.z=-0.1
			self.cmd_vel_pub.publish(self.twist)
			
		cv2.imshow("image window", res)
#create variables only on the first pass
if FIRST:
	global WALL
	WALL = False
	print 'wall'
	print WALL
	time.sleep(5)
	#FIRST = False

rospy.init_node('follower')

follower = Follower()
rospy.spin()
# END ALL
cv2.destroyAllWindows()

