import tf
import rospy
from UAVController import UAVController



class PrintLoc(UAVController):
	def __init__(self):
		return super(PrintLoc, self).__init__()


	def user_control_logic(self):
		info=self.parse_local_position("e")
		rospy.info(str(info))
		rospy.sleep(0.2)



	def user_control_reset(self):
		rospy.loginfo("user program reset!")

	def parse_local_position(self, mode="q"):
		local_position=self.ros_data["local_position"]
		rx=-local_position.pose.position.x
		ry=-local_position.pose.position.y
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


		return condition

	def run(self):
		state = "init"
		while not rospy.is_shutdown():

			if state == "init":
				self.register()
				self.user_control_init()
				self.reset()
				state="2"
			if state=="2":
				ret = self.is_start_ready()
				if ret:
					info=self.parse_local_position("e")
					rospy.loginfo("position:" +  str(info[0:3]))
					rospy.loginfo("roll:" +  str(info[3]))
					rospy.loginfo("pitch:" +  str(info[4]))
					rospy.loginfo("yaw:" +  str(info[5]))
					rospy.sleep(0.2)

if __name__ == "__main__":
	prog = PrintLoc()
	prog.run()
