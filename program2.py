import rospy
from UAVController import UAVController
from mavros_msgs.msg import AttitudeTarget
from sac import *
from collections import defaultdict
from obstacle_description import bbox, wall
import numpy as np
from pid import PID
import time
from matplotlib import pyplot as plt
import tf
import os
import pandas

def rad(x): return x/180.0*np.pi
class Program2(UAVController):
	def __init__(self):
		return super(Program2,self).__init__()

	def check_collision(self):
		x,y,z,r,p,y=self.parse_local_position(mode="e")
		xyz, rpy_eular=(x,y,z), (r,p,y)
		uavbb=bbox(b_center=xyz, 
					bsize_ckg=(0.4,0.4,0.235),
					brpy=rpy_eular)
		return uavbb.is_crush(self.wall)

	#rewrite this function to deal with the inversed axis
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
	#rewrite this function to deal with the inversed axis
	def parse_velocity(self):
		wx=self.ros_data["velocity"].twist.angular.x
		wy=self.ros_data["velocity"].twist.angular.y
		wz=self.ros_data["velocity"].twist.angular.z
		vx=-self.ros_data["velocity"].twist.linear.x
		vy=-self.ros_data["velocity"].twist.linear.y
		vz=self.ros_data["velocity"].twist.linear.z
		return (vx,vy,vz,wx,wy,wz)

	def get_state(self):
		s=[]
		to_pi=lambda m: m-2*np.pi if m>np.pi else m 
		x,y,z,r,p,y=self.parse_local_position(mode="e")
		vx,vy,vz,wx,wy,wz=self.parse_velocity()
		
		s.append((self.target_pos[0]-x)/10.0) #xyz
		s.append((self.target_pos[1]-y)/10.0) #xyz
		s.append((self.target_pos[2]-z)/10.0) #xyz
		s.append(to_pi(self.hole_angle-r)) #xyz
		s.append(to_pi(0-p)) #xyz
		s.append(vx) 
		s.append(vy) 
		s.append(vz) 
		s.append(wx) 
		s.append(wy) 

		return np.array(s)

	def att_controller(self,r=0,p=0,y=0,z=2):
		att=AttitudeTarget()
		q = tf.transformations.quaternion_from_euler(r, p, y)
		att.orientation.x=q[0]
		att.orientation.y=q[1]
		att.orientation.z=q[2]
		att.orientation.w=q[3]
		try:
			rx,ry,rz,r,p,y=self.parse_local_position(mode="e")
			param=1.0/(np.cos(r)*np.cos(p))
		except Exception as e:
			param=1
		finally:
			att.thrust=self.pidz.step(z-rz)*param

		self.att_setpoint_pub.publish(att)
	
	def check_goal(self):
		x,y,z,_,_,_,_=self.parse_local_position()
		return x>2.25
	
	def check_timeout(self):
		return rospy.get_time()-self.acrobat_start_time>3

	def decision_making(self):
		current_time=rospy.get_time()
		mode, ctrl_cmd, done="",{}, False
		if self.state=="takeoff":
			mode, ctrl_cmd="position",{"x":0,"y":0,"z":self.height}
			rx,ry,rz,_,_,_,_=self.parse_local_position()
			rospy.loginfo_throttle(1,"taking off, current position is %s" % str((rx,ry,rz)) )

			if current_time>self.control_start_time+10:
				rospy.loginfo("switching to attitude control!")
				self.state="switch_att_control"

		elif self.state=="switch_att_control":
			mode, ctrl_cmd="attitude",{"r":0,"p":0,"y":0,"z":self.height}
			if current_time>self.control_start_time+20:
				x,y,z,_,_,_,_=self.parse_local_position()
				self.acrobat_cmd={"r":0,"p":0,"y":0,"z":z}
				self.acrobat_start_time=rospy.get_time()
				rospy.loginfo("Now acrobat motion!")
				self.state="acrobat"

		elif self.state=="acrobat":
			state=self.get_state()
			action = policy_net.get_action(state)
			self.acrobat_cmd["r"]+=(action[0]*0.05)
			self.acrobat_cmd["p"]+=(action[1]*0.05)
			self.acrobat_cmd["z"]+=(action[2]*0.01)
			self.acrobat_cmd["r"]=np.clip(self.acrobat_cmd["r"],-0.55,0.55)
			self.acrobat_cmd["p"]=np.clip(self.acrobat_cmd["p"],-0.55,0.55)
			self.acrobat_cmd["z"]=np.clip(self.acrobat_cmd["z"],self.height-1,self.height+1)
			mode, ctrl_cmd="attitude",self.acrobat_cmd
			if self.check_collision():
				self.state="finish"
				rospy.loginfo("collision! acrobat over!")
			elif self.check_goal():
				self.state="finish"
				rospy.loginfo("goal! acrobat win!")
			elif self.check_timeout():
				self.state="finish"
				rospy.loginfo("time out! acrobat over!")

			self.log()
		elif self.state=="finish":
			done=True
		
		return mode, ctrl_cmd, done

	def log(self,**kwargs):
		for k,v in kwargs.items():
			self.data_album[k].append(v)

		x,y,z,r,p,y=self.parse_local_position(mode="e")
		vx,vy,vz,wx,wy,wz=self.parse_velocity()
		ax,ay,az=self.parse_linear_acc()

		self.data_album["time"].append(rospy.get_time()-self.acrobat_start_time)
		self.data_album["action_r"].append(self.acrobat_cmd["r"])
		self.data_album["action_p"].append(self.acrobat_cmd["p"])
		self.data_album["action_z"].append(self.acrobat_cmd["z"])
		self.data_album["al_roll"].append(r)
		self.data_album["al_pitch"].append(p)
		self.data_album["al_yaw"].append(y)
		self.data_album["pos_x"].append(x)
		self.data_album["pos_y"].append(y)
		self.data_album["pos_z"].append(z)
		self.data_album["velocity_x"].append(vx)
		self.data_album["velocity_y"].append(vy)
		self.data_album["velocity_z"].append(vz)
		self.data_album["linear_acc_bodyx"].append(ax)
		self.data_album["linear_acc_bodyy"].append(ay)
		self.data_album["linear_acc_bodyz"].append(az)
		self.data_album["angular_x"].append(wx)
		self.data_album["angular_y"].append(wy)
		self.data_album["angular_z"].append(wz)

	def save_csv(self):
		if not os.path.exists("./csv"):
			os.makedirs("./csv")	
		file_name="csv/"+time.strftime("%d-%H:%M:%S",time.localtime())+".csv"
		df=pandas.DataFrame(self.data_album)
		df.to_csv(file_name)


			

	def render(self):
		if not os.path.exists("./img_evaluation"):
			os.makedirs("./img_evaluation")
		file_name_2d="img_evaluation/"+time.strftime("%d-%H:%M:%S",time.localtime())+"2d.png"
		file_name_3d="img_evaluation/"+time.strftime("%d-%H:%M:%S",time.localtime())+"3d.png"

		x_name=self.data_album.keys()
		col_num, rol_num=3, (len(x_name)-1)/3+1
		fig=plt.figure(figsize=(10,15))
		count=1
		for xn in sorted(x_name):
			if xn=="time" or xn=="pos_3d":
				continue
			ax=plt.subplot(rol_num,col_num,count)
			ax.plot(self.data_album["time"],self.data_album[xn],label=xn,marker='o',markersize=2)
			ax.autoscale()
			ax.ticklabel_format(style='plain',useOffset=False)
			plt.legend()
			count+=1
		fig.savefig(file_name_2d)

	def execution(self,mode, ctrl_cmd):
		if mode=="position":
			self.set_local_position(**ctrl_cmd)

		if mode=="attitude":
			self.att_controller(**ctrl_cmd)

		rospy.sleep(0.02)

	def user_control_reset(self):
		rospy.loginfo("user program reset!")
		load_all_models()
		self.height=2
		self.state="takeoff"
		self.time_cnt=0
		self.control_start_time=rospy.get_time()
		self.pidz=PID(0.4,0.01,3,0,1,0.5)
		self.acrobat_cmd={"r":0,"p":0,"y":0,"z":self.height}
		self.data_album=defaultdict(list)
		self.target_pos=(2.25,0,self.height)
		self.wall_dis, self.hole_w, self.hole_h, self.hole_angle=2,1.0,0.35,rad(20)
		self.wall=wall(wall_pos=self.wall_dis,
						hole_center_p=(0,self.height),
						hole_w=self.hole_w,
						hole_h=self.hole_h,
						hole_angle=self.hole_angle)

		

	def user_control_logic(self):

		self.mode, self.ctrl_cmd, self.done=self.decision_making()
		
		# for 50hz or faster control..

		if not self.done:
			self.execution(self.mode, self.ctrl_cmd)

		self.time_cnt+=1
		return self.done

	def user_control_end(self):
		self.render()
		self.save_csv()
		rospy.loginfo("user program ended!")
#  roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

if __name__ == "__main__":

	print("cuda: ", use_cuda)
	prog = Program2()
	prog.run()
	
