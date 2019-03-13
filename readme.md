# REAL FLIGHT VALIDATION
=========================
this repo contains the XIAO Chenxi's validation framework of his RL algorithm. This framework also facilitates the fast development of PX4 control nodes. Program1.py for the usage demostration. 

Note that the inner coordinate (x,y,z) showed in program2 is in an abnormal axis:
- [x] x = -x_vicon
- [x] y = -y_vicon
- [x] z = z_vicon
=========================
how to run this program at the takeoff location of (-1, -1, 1.4):

```
python program2.py -x -1 -y -1 -z 1.4
```
The takeoff location is (1, 1, 1.4) in the vicon axis.


