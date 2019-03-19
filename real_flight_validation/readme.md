# REAL FLIGHT VALIDATION

this repo contains the XIAO Chenxi's validation framework of his RL algorithm. This framework also facilitates the fast development of PX4 control nodes. Program1.py for the usage demostration. 

Note that the inner coordinate (x,y,z) showed in program2 is in an abnormal axis:
```
x = -x_vicon
y = -y_vicon
z = z_vicon
```
how to run this program by taking off at (-1, -1, 1.4):

```
python pass_hole.py -x -1 -y -1 -z 1.4
```
The hovering position is (1, 1, 1.4) in the vicon axis.

# Program list:
run_square.py: a demo for running a square
pass_hole.py: RL passing hole

# Operation
after starting the program:
1) arm
2) switch to offboard
Then you can see the quadrotor taking off.

# Simulation
copy iris.sdf to src/Firmware/Tools/sitl_gazebo/models/iris. replace the old file.
My gazebo version is 7.0.0. If problems about parsing uav parameters are found, try to do changes to your original iris.sdf according to my version.

# QGC parameter
In simulation, you can use qgc_param.params as the parameter, which can work together with iris.sdf provided above.

# Note
Not px4 firmware of all version can support attitude control. If you find problem about this, try to switch your px4 firmware to other versions!



