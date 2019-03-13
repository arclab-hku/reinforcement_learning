
import rospy
from UAVController import UAVController



class Program1(UAVController):
	def __init__(self):
		return super(Program1, self).__init__()


	def user_control_logic(self):
		self.set_local_position(0,0,2)
		rospy.sleep(0.01)
		return False

	def user_control_reset(self):
		rospy.loginfo("user program reset!")


		
if __name__ == "__main__":
	prog = Program1()
	prog.run()
