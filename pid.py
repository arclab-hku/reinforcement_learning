import numpy as np
class PID:
    def __init__(self,kp,ki,kd,out_min,out_max,out_init=None):
        self.kp=kp
        self.ki=ki
        self.kd=kd
#         self.derv_ine=derv_ine
        self.out_max=out_max
        self.out_min=out_min
        self.out_init=out_init
        self.out=self.out_min if self.out_init is None else self.out_init
        self.derv=0
        self.init=0
        self.err=[0,0,0]
        self.kp_debug=0
        self.ki_debug=0
        self.kd_debug=0
        
    def step(self,cur_err):
        if self.init==0:
            self.err=[cur_err,0,0] 
            self.out=self.out_min if self.out_init is None else self.out_init
        else:
            self.err=[cur_err,self.err[0],self.err[1]]
        self.init+=1

        self.out+=self.kp*(self.err[0]-self.err[1])
        self.out+=self.kd*(self.err[0]-2*self.err[1]+self.err[2])
        if self.out_min<self.out<self.out_max :
            self.out+=self.ki*self.err[0]
            
        return np.clip(self.out,self.out_min,self.out_max)
        
    def reset(self):
        self.init=0