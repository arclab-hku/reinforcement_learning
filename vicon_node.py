
import rospy
from UAVController import UAVController
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf
from collections import deque
from geometry_msgs.msg import PoseStamped, TwistStamped

class Vicon_Node(object):
	def __init__(self):
		rospy.init_node('vicon_preprocessor')
		rospy.loginfo("vicon_preprocessor started!")
		self.vicon_sub = rospy.Subscriber('mavros/vision_pose/pose',
										  TransformStamped,
										  self.pose_recv_callback)
		self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
											  PoseStamped,
											  self.local_position_callback)

		self.recv_cnt=0
		self.recv_cnt_local_pos=0
		self.data_buf = deque(maxlen=10)
		self.local_position=PoseStamped()

	def pose_recv_callback(self, data):
		self.recv_cnt+=1
		
		self.data_buf.append(data)

	def local_position_callback(self, data):
		self.local_position = data
		self.recv_cnt_local_pos+=1

	def unpack_data(self, data):
		x = data.transform.translation.x
		y = data.transform.translation.y
		z = data.transform.translation.z
		q1 = data.transform.rotation.x
		q2 = data.transform.rotation.y
		q3 = data.transform.rotation.z
		q4 = data.transform.rotation.w
		time=data.header.stamp
		time=rospy.Time(time.secs,time.nsecs).to_time()

		rpy_eular = tf.transformations.euler_from_quaternion((q1, q2, q3, q4))
		return (time, (x, y, z), rpy_eular)
	
	def process_data(self):
		pass

	def run(self):
		# try:
		rate=rospy.Rate(50)
		inited=0

		while not rospy.is_shutdown():
			if len(self.data_buf)>0:
				data=self.data_buf[-1]
				time, pos, ang=self.unpack_data(data)
				delta_t=rospy.get_time()-time
				if delta_t>0.1:
					# only 2 appears..
					rospy.logwarn("no vicon data arrival for too long! (2) " + str(delta_t) +" "+str(self.recv_cnt))
			local_pos_time=self.local_position.header.stamp
			time=rospy.Time(local_pos_time.secs,local_pos_time.nsecs).to_time()
			delta_t=rospy.get_time()-time
			# print(time)
			if delta_t>0.1:
				rospy.logwarn("no local_pose data arrival for too long! (2) " + str(delta_t) +" "+str(self.recv_cnt_local_pos))

			rate.sleep()
	


if __name__ == "__main__":
	prog = Vicon_Node()
	prog.run()

