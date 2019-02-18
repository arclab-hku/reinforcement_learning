import os
import subprocess
import rospy
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from geometry_msgs.msg import Twist, Vector3Stamped, Pose,Point,Quaternion,Vector3
from visualization_msgs.msg import MarkerArray,Marker
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header
from utils import *
import tf
from pid import PID
import numpy as np

class ROSNode:
	"""StateMachine: flight routinue control"""
	def __init__(self,Height=10):

		self.rate=25
		self.rateobj = None
		self.set_arm=None
		self.set_mode=None
		self.L=1
		self.H=Height	
		self.local_pos_sub = None
		self.local_position = PoseStamped()
		self.velocityobj=TwistStamped()
		self.imu_data=Imu()
		self.set_arm= None
		self.set_mode = None
		self.pos_setpoint_pub = None
		self.target_odom_pub=None
		self.setpoint_odom_pub=None


	def ros_delay(self,sec):
		rospy.sleep(sec)
		# for i in range(int(sec*self.rate+0.5)):
		# 	# rospy.loginfo("delaying" )
		# 	self.rateobj.sleep()
	def ros_interact_step(self,given_rate=None):
		if given_rate is not None:
			rospy.sleep(1.0/given_rate)
		else:
			rospy.sleep(1.0/self.rate)

		
	def start_sim(self):
		self.err_cmd=0

		self.mavros = subprocess.Popen([r'roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" > /dev/null '],shell=True)
		self.ros_delay(2)
		self.gazebo = subprocess.Popen([r"cd ~/src/Firmware/ && HEADLESS=1 make posix_sitl_default gazebo > /dev/null"],shell=True)
		# offb = subprocess.Popen([r'roslaunch offb offb_local.launch'],shell=True)
		# rviz = subprocess.Popen([r'rosrun rviz rviz'],shell=True)
		rospy.loginfo("start sim!")

	def stop_sim(self):

		subprocess.call([r'killall -9 mavros'],shell=True)
		subprocess.call([r'killall -9 px4'],shell=True)
		# subprocess.call([r'pkill rviz'],shell=True)
		subprocess.call([r'killall -9 gzserver'],shell=True)	
		if hasattr(self,"mavros"):
			self.mavros.kill()
		if hasattr(self,"gazebo"):
			self.gazebo.kill()	

	def reset_sim(self):
		rospy.loginfo("============reset simulation===============")
		self.stop_sim()
		self.start_sim()
	
	def return_zero(self,variance=0.05):
		print("=======SOFT RESET=======")
		cnt=0
		zeropoint = np.random.normal(0, variance, 2)
		state=0
		while True:
			if state==0:
				self.moveto(zeropoint[0],zeropoint[1],self.H)
				self.ros_delay(0.1)
				cnt+=1
				rx,ry,rz,_,_,_,_=self.get_uav_pos()
				rospy.loginfo_throttle(1,(rx,ry,rz))

				if np.linalg.norm([rx-zeropoint[0],ry-zeropoint[1],rz-self.H])<0.3:
					state=1
					cnt=0
					print("now switch to attitude control")
				if cnt>500:
					return False
			elif state==1:
				self.set_attitude(0,0,0,self.H)
				self.ros_delay(0.05)
				cnt+=1
				rx,ry,rz,_,_,_,_=self.get_uav_pos()
				rospy.loginfo_throttle(1,(rx,ry,rz))
				if cnt>100 and rx<1.5: return True
				if rx>1.5: return False

				
	def publish_holemarker(self,pos_arr):
		markerArray = MarkerArray()
		i=0

		for p in pos_arr:
			
			i+=1
			marker = Marker()
			marker.id=i
			marker.header.frame_id = "map"
			marker.type = marker.SPHERE
			marker.scale.x = 0.1
			marker.scale.y = 0.1
			marker.scale.z = 0.1
			marker.color.a = 1.0
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.ns = "my_namespace"
			marker.action = marker.ADD
			marker.pose.orientation.w = 1.0
			marker.pose.position.x = p[0]
			marker.pose.position.y = p[1]
			marker.pose.position.z = p[2]
			markerArray.markers.append(marker)

		self.hole_publisher.publish(markerArray)
	
	def publish_UAV_BB(self):
		x,y,z,qx,qy,qz,qw=self.get_uav_pos()
		marker = Marker()
		marker.header.frame_id = "map"
		marker.type = marker.CUBE
		marker.id=876

		marker.ns = "my_namespace"
		marker.action = marker.ADD
		marker.scale.x = 0.5
		marker.scale.y = 0.5
		marker.scale.z = 0.12
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 1.0
		marker.pose.orientation.x=qx
		marker.pose.orientation.y=qy
		marker.pose.orientation.z=qz
		marker.pose.orientation.w = qw
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = z		
		self.UAVBB_publisher.publish(marker)

	def publish_target_point(self,x,y,z):
		odom=Odometry()

		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "map"
    	
    	# set the position
		odom.pose.pose = Pose(Point(x, y, z), Quaternion(0,0,0,1))

    	# set the velocity
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

		self.target_odom_pub.publish(odom)	

	def publish_setpoint_point(self,x,y,z):
		odom=Odometry()

		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "map"
    	
    	# set the position
		odom.pose.pose = Pose(Point(x, y, z), Quaternion(0,0,0,1))

    	# set the velocity
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

		self.setpoint_odom_pub.publish(odom)	
	
	def set_attitude(self,r,p,y,z,mode="height"):
		att=AttitudeTarget()
		q = tf.transformations.quaternion_from_euler(r, p, y)
		att.orientation.x=q[0]
		att.orientation.y=q[1]
		att.orientation.z=q[2]
		att.orientation.w=q[3]
		rx,ry,rz,qx,qy,qz,qw=self.get_uav_pos()
		if mode=="height":
			try:
				rpy=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
				param=1.0/(np.cos(rpy[0])*np.cos(rpy[1]))
			except:
				param=1
			finally:
				att.thrust=self.pidz.step(z-rz)*param
		else:
			att.thrust=z
		self.att_setpoint_pub.publish(att)


	def set_velocity(self,vx,vy,vz):
		vel = TwistStamped()
		vel.twist.linear.x = vx
		vel.twist.linear.y = vy
		vel.twist.linear.z = vz
		vel.header = Header()
		vel.header.frame_id = "map"
		vel.header.stamp = rospy.Time.now()
		self.pos_setvel_pub.publish(vel)
		

	def moveto(self,x,y,z):
		
		self.publish_setpoint_point(x,y,z)
		pos = PoseStamped()
		pos.pose.position.x = x
		pos.pose.position.y = y
		pos.pose.position.z = z
		pos.header = Header()
		pos.header.frame_id = "map"
		pos.header.stamp = rospy.Time.now()
		# rospy.loginfo_throttle(0.41,pos)
		# rospy.loginfo(pos)
		self.pos_setpoint_pub.publish(pos)
		# self.publish_target_odom.publish(x,y,z)

	def local_position_callback(self,data):
		self.local_position = data	
		# rospy.loginfo_throttle(1,data)
	
	# def get_angular_v(self):
	# 	return [self.imu_data.angular_velocity.x, self.imu_data.angular_velocity.y, self.imu_data.angular_velocity.z]
		# return tf.transformations.euler_from_quaternion(self.imu_data.angular_velocity)
	def get_angular_v(self):
		return [self.velocityobj.twist.angular.x, self.velocityobj.twist.angular.y, self.velocityobj.twist.angular.z]
	
	def get_linear_v(self):
		return [self.velocityobj.twist.linear.x, self.velocityobj.twist.linear.y, self.velocityobj.twist.linear.z]
	
	def velocity_callback(self,data):
		self.velocityobj=data

	def Imu_callback(self,data):
		self.imu_data=data

	def get_uav_state12(self):
		rx,ry,rz,qx,qy,qz,qw=self.get_uav_pos()
		rpy=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
		ang_v=self.get_angular_v()
		v=self.get_linear_v()
		return [-rx,-ry,rz,rpy[0],rpy[1],rpy[2],-v[0],-v[1],v[2],ang_v[0],ang_v[1],ang_v[2]]



	def get_uav_pos(self):
		rx=self.local_position.pose.position.x
		ry=self.local_position.pose.position.y
		rz=self.local_position.pose.position.z
		
		qx=self.local_position.pose.orientation.x
		qy=self.local_position.pose.orientation.y
		qz=self.local_position.pose.orientation.z
		qw=self.local_position.pose.orientation.w

		return rx,ry,rz,qx,qy,qz,qw

	def hover(self, sec):
		for i in range(int(sec/self.rate)):
			self.moveto(0,0,self.H)
			self.ros_interact_step()

	def flight_init(self):
		#https://github.com/ethz-asl/rotors_simulator/blob/master/rqt_rotors/src/rqt_rotors/hil_plugin.py
		#https://github.com/PX4/Firmware/blob/master/integrationtests/python_src/px4_it/mavros/mavros_offboard_posctl_test.py
		#https://github.com/PX4/Firmware/blob/master/integrationtests/python_src/px4_it/mavros/mavros_test_common.py
		rospy.loginfo("============flight_init===============")
		for i in range(5):

			try:
		
				for i in range(10):
					self.moveto(0,0,self.H)
	
				self.ros_delay(0.1)
				res = self.set_arm(True)
				if not res.success:
					raise rospy.ServiceException("failed to send ARM command")
					continue
				res = self.set_mode(0,"OFFBOARD")
				if not res.mode_sent:
					raise rospy.ServiceException("failed to send OFFBOARD command")
					continue

			except rospy.ServiceException as e:
				rospy.logerr(e)
				continue
			return True

		
			
		return False

	def lightweight_position_control(self,tarpos,Kp=1,maxspd=2):
		vx=Kp*(tarpos[0]-self.get_uav_pos()[0])
		vy=Kp*(tarpos[1]-self.get_uav_pos()[1])
		vz=Kp*(tarpos[2]-self.get_uav_pos()[2])

		norm=np.linalg.norm([vx,vy,vz])
		if norm>maxspd:
			vx=vx*maxspd/norm
			vy=vy*maxspd/norm
			vz=vz*maxspd/norm

		#self.set_velocity(vx,vy,vz)
		return (vx,vy,vz)







	def start(self):
		print("calling start")
		#references: 
		rospy.init_node('ROSNode')
		rospy.loginfo("ROSNode started!")
		self.rateobj=rospy.Rate(self.rate)
		self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', 
												PoseStamped,
												self.local_position_callback)
		self.local_imu_sub = rospy.Subscriber('mavros/imu/data', 
												Imu,
												self.Imu_callback)
		self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity', 
												TwistStamped,
												self.velocity_callback)
		self.set_arm=rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
		self.pos_setvel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)
		self.target_odom_pub = rospy.Publisher("target_odom", Odometry, queue_size=50)
		self.setpoint_odom_pub = rospy.Publisher("setpoint_odom", Odometry, queue_size=50)
		self.hole_publisher=rospy.Publisher("hole_markers",MarkerArray, queue_size=10)
		self.UAVBB_publisher=rospy.Publisher("UAVBB_marker",Marker, queue_size=10)
		self.att_setpoint_pub=rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

		self.local_position=None
		self.pidz=PID(0.4,0.03,3,0,1)
		

		state = 0

		while not rospy.is_shutdown():

			if state==0:
				self.reset_sim()
				self.ros_delay(10)#delay should be long enough... or potential bug..
				res = self.flight_init()
				if res:
					rospy.loginfo("flight init ok!")
					state = 1
				else:
					state=0 #do nothing
					# self.reset_sim()
					# self.ros_delay(5)
			if state==1:
				# self.Env.moveto(0,0,self.H)

				cnt=0
				print("taking off....")
				while True:
					try:
						rx,ry,rz,_,_,_,_=self.get_uav_pos()
						# self.set_velocity(0,0,10)
						self.moveto(0,0,self.H)
					except:
						cnt+=1
						print("error 0999!!!!!")
						continue
						
					if np.linalg.norm([rx,ry,rz-self.H])<0.3:
						state=2
						print("now switching control mode...")
						cnt=0
						break
					else:
						cnt+=1
					# print("taking off....",[rx,ry,rz])
					
					self.ros_delay(0.05)
					rospy.loginfo_throttle(1,(rx,ry,rz))

					if cnt>=300:
						state=0
						break
			if state==2:
				self.set_attitude(0,0,0,self.H)
				cnt+=1
				rx,ry,rz,_,_,_,_=self.get_uav_pos()
				rospy.loginfo_throttle(1,(rx,ry,rz))
				if cnt>300: state=99

			if state==99:
				rospy.loginfo("============take off ok!============")
				return True
	

			self.rateobj.sleep()
					
	def start_lightweight_pid(self):
		print("calling start")
		#references: 
		rospy.init_node('ROSNode')
		rospy.loginfo("ROSNode started!")
		self.rateobj=rospy.Rate(self.rate)
		self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', 
												PoseStamped,
												self.local_position_callback)
		self.local_pos_sub = rospy.Subscriber('mavros/imu_data', 
												PoseStamped,
												self.local_position_callback)

		self.set_arm=rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
		self.pos_setvel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)
		self.att_setpoint_pub=rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
		self.target_odom_pub = rospy.Publisher("target_odom", Odometry, queue_size=50)
		self.setpoint_odom_pub = rospy.Publisher("setpoint_odom", Odometry, queue_size=50)
		self.hole_publisher=rospy.Publisher("hole_markers",MarkerArray)
		
		self.local_position=None


		state = 0

		while not rospy.is_shutdown():

			if state==0:
				self.reset_sim()
				self.ros_delay(10)#delay should be long enough... or potential bug..
				res = self.flight_init()
				if res:
					rospy.loginfo("flight init ok!")
					state = 1
				else:
					state=0 #do nothing
					# self.reset_sim()
					# self.ros_delay(5)
			if state==1:
				# self.Env.moveto(0,0,self.H)

				cnt=0
				print("taking off....")
				while True:
					try:
						rx,ry,rz,_,_,_,_=self.get_uav_pos()
						# self.set_velocity(0,0,10)
						self.moveto(0,0,self.H)
					except:
						cnt+=1
						print("error 0999!!!!!")
						continue
					
					if np.linalg.norm([rx,ry,rz-self.H])<0.3:
						state=2
						
						break
					else:
						cnt+=1
					# print("taking off....",[rx,ry,rz])
					
					self.ros_delay(0.05)
					rospy.loginfo_throttle(1,(rx,ry,rz))

					if cnt>=300:
						state=0
						break
			if state==2:
				self.lightweight_position_control((-1,-2,5))
				self.ros_delay(0.05)
				rx,ry,rz,_,_,_,_=self.get_uav_pos()
				rospy.loginfo_throttle(1,(rx,ry,rz))


			if state==99:
				rospy.loginfo("============take off ok!============")
				return True
	

			self.rateobj.sleep()

	def test(self):
		self.reset_sim()
		rospy.init_node('ROSNode')
		rospy.loginfo("ROSNode started!")
		self.rateobj=rospy.Rate(1)
		while True:

			seconds = rospy.get_time()
			clk=time.clock()
			self.rateobj.sleep()
			x=rospy.get_time()-seconds
			x2=time.clock()-clk
			print("**************",x,x2,"**************")
		self.stop_sim()






if __name__ == '__main__':
	print("tests!")
	s=ROSNode()
	s.start()
	i=0
	while True:
		s.set_attitude(0.1,0,0,10)
		# s.moveto(0,0,10)
		s.rateobj.sleep()
		# i+=1
		# s.return_zero()
		# print(s.get_uav_state12())
	
