import rospy
from UAVController import UAVController
from mavros_msgs.msg import AttitudeTarget
from collections import defaultdict
from obstacle_description import bbox, wall
import numpy as np
from pid import PID
import time
import matplotlib
from matplotlib import pyplot as plt
import tf
import os
import pandas

matplotlib.use("Agg")

def rad(x): return x/180.0*np.pi

class Program3(UAVController):
	def __init__(self):
		super(Program3,self).__init__()



	def att_controller(self,r=0,p=0,y=0,z=2):
		att=AttitudeTarget()
		q = tf.transformations.quaternion_from_euler(r, p, y)
		att.orientation.x=q[0]
		att.orientation.y=q[1]
		att.orientation.z=q[2]
		att.orientation.w=q[3]
		rx,ry,rz,r,p,y=self.parse_local_position(mode="e")
		vx,vy,vz,wx,wy,wz=self.parse_velocity()
		try:

			param=1.0/(np.cos(r)*np.cos(p))
		except Exception as e:
			param=1
		finally:
			vzt=self.pidz.step(z-rz)
			att.thrust=self.pidzv.step(vzt-vz)*param
			# for plot only..
			self.thrust=att.thrust
			self.pidzv_out=self.pidzv.out
			self.pidz_out=self.pidz.out
	

		self.att_setpoint_pub.publish(att)
	
	def set_roll_pitch_zero(self,roll,pitch):
		self.roll_zero=roll
		self.pitch_zero=pitch

	

	def decision_making(self):

		current_time=rospy.get_time()
		mode, ctrl_cmd, done="",{}, False
		

		if self.state=="takeoff":
			mode, ctrl_cmd="position",{"x":0,"y":0,"z":self.height}
			rx,ry,rz,r,p,y=self.parse_local_position(mode="e")
			rospy.loginfo_throttle(1,"taking off, current position is %s" % str((rx,ry,rz)) )
			if current_time>self.control_start_time+5:
				self.sample_roll_array.append(r)
				self.sample_pitch_array.append(p)
			if current_time>self.control_start_time+10:
				rospy.loginfo("switching to attitude control!")
				self.state="switch_att_control"
				self.switch_att_time=rospy.get_time()
				roll=np.mean(self.sample_roll_array)
				pitch=np.mean(self.sample_pitch_array)
				self.set_roll_pitch_zero(roll,pitch)
				rospy.loginfo("roll: " + str(roll)+", pitch: " + str(pitch))

		elif self.state=="switch_att_control":
			mode, ctrl_cmd="attitude",{"r":self.roll_zero,"p":self.pitch_zero,"y":0,"z":self.height}
			self.acrobat_cmd={"r":self.roll_zero,"p":self.pitch_zero,"y":0,"z":self.height}
			if current_time>self.control_start_time+13:
				x,y,z,_,_,_,_=self.parse_local_position() 
				#z start from current position: for a stable transfer..
				self.acrobat_cmd={"r":self.roll_zero,"p":self.pitch_zero,"y":0,"z":self.height}
				self.acrobat_start_time=rospy.get_time()
				rospy.loginfo("Now acrobat motion!")
				self.state="acrobat"
			self.log()

		elif self.state=="acrobat":
			t=rospy.get_time()-self.acrobat_start_time
			self.acrobat_cmd["r"]=self.roll_zero #-0.05*np.sin(t*5)
			self.acrobat_cmd["p"]=self.pitch_zero# self.pitch_zero
			self.acrobat_cmd["y"]=0
			self.acrobat_cmd["z"]=self.height# self.height+0.5*np.sin(t*2)

			# self.acrobat_cmd["r"]=np.clip(self.acrobat_cmd["r"],-0.55,0.55)
			# self.acrobat_cmd["p"]=np.clip(self.acrobat_cmd["p"],-0.55,0.55)
			# self.acrobat_cmd["z"]=np.clip(self.acrobat_cmd["z"],self.height-1,self.height+1)
			mode, ctrl_cmd="attitude",self.acrobat_cmd
			x,y,z,_,_,_,_=self.parse_local_position() 
			if np.linalg.norm([x,y,z-self.height])>1.5:
				self.state="return_zero"
				self.return_zero_start_time=rospy.get_time()
				rospy.loginfo("too far from center! return zero!")

			self.log()
			if rospy.get_time()-self.acrobat_start_time>10:
				self.state="prepare_landing"
				self.prepare_landing_time=rospy.get_time()
				rospy.loginfo("motion over! prepare landing! ")

		elif self.state=="prepare_landing":
			mode, ctrl_cmd="position",{"x":0,"y":0,"z":self.height}
			if rospy.get_time()-self.prepare_landing_time>5:
				done=True

		elif self.state=="return_zero":
			mode, ctrl_cmd="position",{"x":0,"y":0,"z":self.height}
			if rospy.get_time()-self.return_zero_start_time>3:
				rospy.loginfo("switching to attitude control!")
				self.state="switch_att_control"

		
		return mode, ctrl_cmd, done

	def log(self,**kwargs):
		for k,v in kwargs.items():
			self.data_album[k].append(v)

		x,y,z,r,p,y=self.parse_local_position(mode="e")
		vx,vy,vz,wx,wy,wz=self.parse_velocity()
		ax,ay,az=self.parse_linear_acc()
		wx2,wy2,wz2=self.parse_angular_velocity_by_local_position()

		self.data_album["time"].append(rospy.get_time()-self.switch_att_time)

		self.data_album["al_roll"].append(r)
		self.data_album["al_pitch"].append(p)
		self.data_album["al_yaw"].append(y)
		self.data_album["pos_x"].append(x)
		self.data_album["pos_y"].append(y)
		self.data_album["pos_z"].append(z)
		self.data_album["action_r"].append(self.acrobat_cmd["r"])
		self.data_album["action_p"].append(self.acrobat_cmd["p"])
		self.data_album["action_y"].append(self.acrobat_cmd["y"])
		self.data_album["action_z"].append(self.acrobat_cmd["z"])
		self.data_album["velocity_x"].append(vx)
		self.data_album["velocity_y"].append(vy)
		self.data_album["velocity_z"].append(vz)
		self.data_album["linear_acc_bodyx"].append(ax)
		self.data_album["linear_acc_bodyy"].append(ay)
		self.data_album["linear_acc_bodyz"].append(az)
		self.data_album["angular_x"].append(wx)
		self.data_album["angular_y"].append(wy)
		self.data_album["angular_z"].append(wz)
		self.data_album["angular_x2"].append(wx2)
		self.data_album["angular_y2"].append(wy2)
		self.data_album["angular_z2"].append(wz2)
		try:
			self.data_album["pidz_out"].append(self.pidz_out)
		except:
			self.data_album["pidz_out"].append(0)
		try:
			self.data_album["pidzv_out"].append(self.pidzv_out)
		except:
			self.data_album["pidzv_out"].append(0)

		# self.data_album["pidzv_out_limited"].append(self.thrust)
		

	def save_csv(self):
		if not os.path.exists("./csv"):
			os.makedirs("./csv")	
		file_name="csv/"+time.strftime("%d-%H:%M:%S",time.localtime())+".csv"
		df=pandas.DataFrame(self.data_album)
		df.to_csv(file_name)


			

	def render(self):
		if not os.path.exists("./pid_debug_img"):
			os.makedirs("./pid_debug_img")
		file_name_2d="pid_debug_img/"+time.strftime("%d-%H:%M:%S",time.localtime())+"2d.png"
		file_name_3d="pid_debug_img/"+time.strftime("%d-%H:%M:%S",time.localtime())+"3d.png"

		x_name=self.data_album.keys()
		col_num, rol_num=3, (len(x_name)-1)/3+1
		fig=plt.figure(figsize=(10,15))
		count=1
		for xn in sorted(x_name):
			if xn=="time" or xn=="pos_3d":
				continue
			ax=plt.subplot(rol_num,col_num,count)
			try:
				ax.plot(self.data_album["time"],self.data_album[xn],label=xn,marker='o',markersize=2)
				ax.autoscale()
				ax.ticklabel_format(style='plain',useOffset=False)
				plt.legend()
			except Exception as e:
				rospy.logwarn(xn+": plot err!")
				print(e)
				ax.plot(self.data_album[xn],label=xn,marker='o',markersize=2)
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

		self.rate.sleep()

	def user_control_reset(self):
		rospy.loginfo("user program reset!")
		
		self.height=1.0
		self.state="takeoff"
		self.time_cnt=0
		self.control_start_time=rospy.get_time()
		self.pidz=PID(kp=1,ki=0.2/50.0,kd=0.006*50,out_min=-1,out_max=3,out_init=0)
		self.pidzv=PID(kp=0.7,ki=0.06/50.0,kd=0.003*50,out_min=0,out_max=1,out_init=0.55)
		self.data_album=defaultdict(list)
		self.sample_roll_array=[]
		self.sample_pitch_array=[]
		self.roll_zero,self.pitch_zero=0,0

	def user_control_logic(self):

		self.mode, self.ctrl_cmd, self.done=self.decision_making()
		self.rate=rospy.Rate(50.0)
		# for 50hz or faster control..

		if not self.done:
			self.execution(self.mode, self.ctrl_cmd)

		self.time_cnt+=1
		return self.done

	def user_control_end(self,mode="normal"):
		self.render()
		self.save_csv()
		rospy.loginfo("user program ended! mode is "+str(mode) )

#  roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

if __name__ == "__main__":
	prog = Program3()
	prog.run()
	
