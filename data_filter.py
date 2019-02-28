import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
data=pd.read_csv("~/catkin_ws/src/real_flight_validation/datas/123.csv")
# print(data["angular_x"])
angular_x_ori=[]
angular_x_f=[]
filter_coff_log=[]
for new_data in data["angular_x"]:
    angular_x_ori.append(new_data)
    if len(angular_x_f)==0: 
        angular_x_f.append(new_data)
        filter_coff_log.append(0)
        continue

    cur_data=angular_x_f[-1]
    filter_coff=0.3
    changed=abs(cur_data-new_data)
    if changed>0.15:
        filter_coff+=(0.9-0.3)*(changed-0.15)/0.3
    filter_coff=np.clip(filter_coff,0.3,0.9)

    tmp=cur_data*filter_coff+new_data*(1-filter_coff)
    angular_x_f.append(tmp)
    filter_coff_log.append(filter_coff)

last_ang=0
ang_speed_calculated=[]
for i,new_data in enumerate(data["al_roll"]):
    if len(ang_speed_calculated)==0: 
        ang_speed_calculated.append(0)
        last_ang=new_data
        filtered_delta=0
        continue

    delta=new_data-last_ang
    filtered_delta=delta*0.4+filtered_delta*0.6
    ang_speed_calculated.append(filtered_delta*50)
    last_ang=new_data
    

plt.plot(angular_x_ori)
plt.plot(angular_x_f)
plt.plot(filter_coff_log)
plt.plot(ang_speed_calculated)
plt.show()