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
python program2.py -x -1 -y -1 -z 1.4
```
The hovering position is (1, 1, 1.4) in the vicon axis.

# Program list:
run_square.py: a demo for running a square

pass_hole.py: RL passing hole



