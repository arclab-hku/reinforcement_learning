import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from geometry_msgs.msg import Twist, Vector3Stamped, Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Header
import numpy as np
import tf

class StateMachineError(Exception):
	pass

class UAVController(object):
	def register(self):
		self.ros_data = {}
		rospy.init_node('UAV_controller')
		rospy.loginfo("UAV_controller started!")
		self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
											  PoseStamped,
											  self.local_position_callback)
		self.local_imu_sub = rospy.Subscriber('mavros/imu/data',
											  Imu,
											  self.Imu_callback)
		self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity',
											  TwistStamped,
											  self.velocity_callback)
		self.state_sub = rospy.Subscriber('mavros/state',
										  State,
										  self.state_callback)
		self.set_arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.set_landing = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.pos_setpoint_pub = rospy.Publisher(
			'mavros/setpoint_position/local', PoseStamped, queue_size=10)
		self.pos_setvel_pub = rospy.Publisher(
			'mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
		self.target_odom_pub = rospy.Publisher(
			"target_odom", Odometry, queue_size=50)
		self.setpoint_odom_pub = rospy.Publisher(
			"setpoint_odom", Odometry, queue_size=50)
		self.hole_publisher = rospy.Publisher(
			"hole_markers", MarkerArray, queue_size=10)
		self.UAVBB_publisher = rospy.Publisher(
			"UAVBB_marker", Marker, queue_size=10)
		self.att_setpoint_pub = rospy.Publisher(
			'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

	def reset(self):
		self.taken_off=False
		self.ros_data = {}
		self.assure_reseted = False
		

	def landing(self):
		rospy.loginfo_throttle(1,"UAV start landing...")
		ret = self.set_landing(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)

		return ret

	def is_user_reset(self):
		state = self.ros_data.get("state", None)
		if state is not None:
			if state.armed==False and state.mode != "OFFBOARD":
				return True
		return False
	
	def prepare_offboard(self):
		for i in range(100):
			self.set_local_position(0,0,1)
			rospy.sleep(-1)

	def is_start_ready(self):
		ret = False

		condition = all(
			[
				self.ros_data.get("local_position", None) is not None,
				self.ros_data.get("imu", None) is not None,
				self.ros_data.get("velocity", None) is not None,
				self.ros_data.get("state", None) is not None,
			]
		)


		if condition:
			if self.ros_data["state"].connected:
				if self.ros_data["state"].mode != "OFFBOARD":
					self.assure_reseted = True

				if self.ros_data["state"].mode == "OFFBOARD" and self.ros_data["state"].armed and self.assure_reseted:
					ret = True
				else:
					rospy.loginfo_throttle(1,"Please meet the takeoff condition first!\n mode:%s, armed:%s"% (self.ros_data["state"].mode,self.ros_data["state"].armed))
			else:
				rospy.logwarn_throttle(1,"mavros connection failure!")
		else:
			rospy.logwarn_throttle(1,"subscribed data has not been fully prepared, waiting for data...")

		return ret

	def safety_monitor(self):
		x=self.ros_data["local_position"].pose.position.x
		y=self.ros_data["local_position"].pose.position.y
		z=self.ros_data["local_position"].pose.position.z
		if z>=0.35:
			self.taken_off=True
		#note that z axis have a hystersis, for the shaking avoidance..


		state=self.ros_data.get("state", None)
		if state is not None:
			if state.armed==False or state.mode!="OFFBOARD":
				rospy.logwarn("user interupted! trying to end this episode...")
				return True
		else:
			rospy.logwarn("abnormal state!") # this should not happen....
			return True

		if self.taken_off and (x<-2.5 or x>2.5 or y<-1.5 or y>1.5 or z<0.25 or z>3):
			rospy.logwarn("land due to safety constraint! pos:%s" % str((x,y,z)) )
			return True

		return False

	def local_position_callback(self, data):
		self.ros_data["local_position"] = data

	def Imu_callback(self, data):
		self.ros_data["imu"] = data

	def velocity_callback(self, data):
		self.ros_data["velocity"] = data

	def state_callback(self, data):
		self.ros_data["state"] = data

	def user_control_logic(self):
		raise NotImplementedError

	def user_control_reset(self):
		raise NotImplementedError
	
	def user_control_end(self,mode="normal"):
		raise NotImplementedError

	def set_local_position(self,x,y,z):
		pos = PoseStamped()
		pos.pose.position.x = x
		pos.pose.position.y = y
		pos.pose.position.z = z
		pos.header = Header()
		pos.header.frame_id = "map"
		pos.header.stamp = rospy.Time.now()
		self.pos_setpoint_pub.publish(pos)
		
	def parse_local_position(self, mode="q"):
		local_position=self.ros_data["local_position"]
		rx=local_position.pose.position.x
		ry=local_position.pose.position.y
		rz=local_position.pose.position.z
		
		qx=local_position.pose.orientation.x
		qy=local_position.pose.orientation.y
		qz=local_position.pose.orientation.z
		qw=local_position.pose.orientation.w

		if mode == "q":
			return (rx,ry,rz,qx,qy,qz,qw)

		elif mode == "e":
			rpy_eular=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
			return (rx,ry,rz)+ rpy_eular
		else:
			raise UnboundLocalError
	
	def parse_velocity(self):
		wx=self.ros_data["velocity"].twist.angular.x
		wy=self.ros_data["velocity"].twist.angular.y
		wz=self.ros_data["velocity"].twist.angular.z
		vx=self.ros_data["velocity"].twist.linear.x
		vy=self.ros_data["velocity"].twist.linear.y
		vz=self.ros_data["velocity"].twist.linear.z
		return (vx,vy,vz,wx,wy,wz)
	
	def parse_linear_acc(self):
		ax=self.ros_data["imu"].linear_acceleration.x
		ay=self.ros_data["imu"].linear_acceleration.y
		az=self.ros_data["imu"].linear_acceleration.z
		return (ax,ay,az)

	def run(self):
		state = "init"
		while True:

			if state != "init" and state!= "wait_signal" and self.is_user_reset():
				state = "reset"
				self.user_control_end(mode="force")
				rospy.loginfo("user reset from the remote controller!")

			if state == "init":
				self.register()
				state = "reset"

			elif state == "reset":
				self.reset()
				state = "wait_signal"

			elif state == "wait_signal":
				rospy.sleep(0.1)
				self.prepare_offboard()
				ret = self.is_start_ready()
				if ret:
					state = "program"
					rospy.loginfo("now starting user program!")
					self.user_control_reset()

			elif state == "program":
				if self.safety_monitor() or self.user_control_logic():
					state = "landing"
					
			elif state == "landing":
				if not self.landing():
					rospy.logwarn("error occured in the landing process!")
					state = "error"
				else:		
					state="finish"
					rospy.loginfo("Task finished! waiting for user control!")

				if state!="landing":
					self.user_control_end()

			elif state=="finish":
				rospy.sleep(0.01)

			elif state == "error":
				rospy.sleep(0.01)
				rospy.logwarn_throttle(
					0.5, "error occured! need manual operation")
			else:
				raise StateMachineError