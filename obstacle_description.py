import numpy as np
from shapely.geometry import Point
from shapely import affinity
from shapely.geometry.polygon import Polygon
import unittest
import matplotlib.pyplot as plt

class wall(object):
    def __init__(self,wall_pos,hole_center_p,hole_w,hole_h,hole_angle):
        self.wall_pos=wall_pos
        polygon = Polygon([(hole_center_p[0]-hole_w/2, hole_center_p[1]-hole_h/2), 
                            (hole_center_p[0]-hole_w/2, hole_center_p[1]+hole_h/2), 
                            (hole_center_p[0]+hole_w/2, hole_center_p[1]+hole_h/2), 
                            (hole_center_p[0]+hole_w/2, hole_center_p[1]-hole_h/2)])
        polygon=affinity.rotate(polygon, hole_angle,use_radians=True, origin='centroid')

        self.hole=polygon


    def in_hole(self,p):
        return self.hole.contains(Point(p[1:]))

    def which_side(self,p):
        return p[0]<self.wall_pos

    def __str__(self):
        return "wall pos:" + str(self.wall_pos)
    
    def get_corner_points(self):
        return self.hole.exterior.coords



class bbox(object):
    class segment(object):
        def __init__(self,p1,p2):
            self.p1=p1
            self.p2=p2
            self.seg_len=np.linalg.norm([x-y for x,y in zip(p2,p1)])
        def __str__(self):
            return "from " +str(self.p1) +"to" + str(self.p2)

        def intersect_wall(self,w):
            p1, p2 = self.p1, self.p2
            a=w.which_side(self.p1)
            b=w.which_side(self.p2)

            if a!=b:
                r=(w.wall_pos-p1[0])/(p2[0]-p1[0])
                l_d1,c1= (p2[1]-p1[1], p1[1]) 
                l_d2,c2= (p2[2]-p1[2], p1[2]) 
                y=r*l_d1+c1
                z=r*l_d2+c2
                return True, (w.wall_pos,y,z)
            return False, None

    def __init__(self,b_center,bsize_ckg,brpy):
        b_center=np.array(b_center)
        dx,dy,dz=[m/2.0 for m in bsize_ckg]
        r_dxyz=[self.rotate((i*dx,j*dy,k*dz),brpy) for i in (-1,1) for j in (-1,1) for k in (-1,1)]
        self.points=[a+b_center for a in r_dxyz]
        self.segs=[ self.segment(p1,p2) for i,p1 in enumerate(self.points) for p2 in self.points[i+1:] ]

    def rotate(self,p,rpy):
        C=np.cos
        S=np.sin
        ii,jj,kk=rpy

        R = [[C(kk) * C(jj), C(kk) * S(jj) * S(ii) - S(kk) * C(ii), C(kk) * S(jj) * C(ii) + S(kk) * S(ii)],
            [S(kk) * C(jj), S(kk) * S(jj) * S(ii) + C(kk) * C(ii), S(kk) * S(jj) * C(ii) - C(kk) * S(ii)],
            [-S(jj), C(jj) * S(ii), C(jj) * C(ii)]]
        return np.array(R).dot(p)

    def is_crush(self,w):
        intersect_points=[]
        for s in self.segs:
            res,p=s.intersect_wall(w)
            if res:
                intersect_points.append(p)
        
        hole_pts=[w.in_hole(p) for p in intersect_points]

        return not all(hole_pts)
    
    def __str__(self):
        return str(self.points)
        