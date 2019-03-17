
import rospy
from UAVController import UAVController



class Program1(UAVController):
	def __init__(self):
		return super(Program1, self).__init__()


	def user_control_logic(self):
		time,period=rospy.get_time(),5
		pts=[(-1, -1), (-1, 1), (1, 1), (1, -1)]
		x,y=pts[int(time/period)%4]
		self.set_local_position(x,y,2)
		rospy.sleep(0.01)
		return time-self.start_time>30

	def user_control_reset(self):
		self.start_time=rospy.get_time()
		rospy.loginfo("user program reset!")


		
if __name__ == "__main__":
	prog = Program1()
	prog.run()
