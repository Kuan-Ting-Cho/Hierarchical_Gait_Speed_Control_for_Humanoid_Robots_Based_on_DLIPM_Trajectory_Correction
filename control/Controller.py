import numpy as np
import transforms3d as tfs
from src.LIPMFunction import InvK
from Simulate.Dataprocess import deg2rad
# from control.Fuzzy import *
import time
def threshold(sig,threshold):
    if sig > threshold:
        sig = threshold
    elif sig < -threshold:
        sig = -threshold

    return sig

def limit(signal,threshold):
    sig=[]
    for i,tmp in enumerate(signal):
        if tmp > threshold:
            sig.append(threshold)
        elif tmp < -threshold:
            sig.append(-threshold)
        else:
            sig.append(tmp)

    return np.array(sig)

def PID_control(kp, kv, ki, qpos, qvel, controller,acc_err,rise):
    signal = []
    for i in range(len(controller)):
        acc_err[i] = acc_err[i] + qpos[i]-controller[i]
        if i == 2 or i == 8 : #OK
            sig=threshold(-kp[2]*(qpos[i]-controller[i])-ki[2]*acc_err[i]-kv[2]*qvel[i],35)
            signal.append(sig)
        elif i == 6 and rise == 2:
            sig=threshold(-kp[1]*(qpos[i]-controller[i])-ki[1]*acc_err[i]-kv[1]*qvel[i],35)
            signal.append(sig)
        elif i == 12 and rise == 1:
            sig=threshold(-kp[1]*(qpos[i]-controller[i])-ki[1]*acc_err[i]-kv[1]*qvel[i],35)
            signal.append(sig)
        else:
            sig=threshold(-kp[0]*(qpos[i]-controller[i])-ki[0]*acc_err[i]-kv[0]*qvel[i],35)
            signal.append(sig)

    return signal,acc_err

# def PD_control(kp, kv, qpos, qvel, controller,rise):
#     signal = []
#     for i in range(len(controller)):
#         if i == 6 and rise == 2:
#             sig=threshold(-kp[1]*(qpos[i]-controller[i])-kv[1]*qvel[i],35)
#             signal.append(sig)
#         elif i == 12 and rise == 1:
#             sig=threshold(-kp[1]*(qpos[i]-controller[i])-kv[1]*qvel[i],35)
#             signal.append(sig)
#         else:
#             sig=threshold(-kp[0]*(qpos[i]-controller[i])-kv[0]*qvel[i],35)
#             signal.append(sig)
#     return signal

def Sole_Status(sensordata):
    #確認腳底板是否觸地(觸地即picth方向需平行)
    parallel=[]
    rise=0
    for idx in range(0,len(sensordata),2):
        if sensordata[idx]!=0 or sensordata[idx+1]!=0:
           parallel.append(1)
        else: 
           parallel.append(0)
    #確認腳是否懸空
    if  parallel[0]+parallel[1]== 0:
        rise = 2   #左腳懸空
    elif parallel[2]+parallel[3]== 0:
        rise = 1   #右腳懸空
    elif parallel[0]+parallel[1]+parallel[2]+parallel[3]==4:
        rise = 0   #DSP
    elif parallel[0]+parallel[1]+parallel[3]==3:
        rise = 3   #Switch l2r
    elif parallel[1]+parallel[2]+parallel[3]==3:
        rise = 4   #Switch r2l
    else:
        rise = 0   #DSP

    return parallel,rise

def Sole_control(model,sole_joint_pos,sensor_pos,parallel):
    desired_angle=[]
    desired_quat=[]
    for idx,state in enumerate(parallel):
        if state==1: #平行，兩邊皆觸地
           if idx==0: #左腳前
              err=sole_joint_pos[0]-(sensor_pos[0]+sensor_pos[1])/2
              rad=np.arcsin(err[0]/0.061)
              rad=threshold(rad,deg2rad(3))
           elif idx==1:#左腳後
              err=sole_joint_pos[1]-(sensor_pos[2]+sensor_pos[3])/2
              rad=np.arcsin(err[0]/0.061)
              rad=threshold(rad,deg2rad(3))
           elif idx==2:#右腳前
              err=sole_joint_pos[2]-(sensor_pos[4]+sensor_pos[5])/2
              rad=np.arcsin(err[0]/0.061)
              rad=threshold(rad,deg2rad(3))
           elif idx==3:#右腳後
              err=sole_joint_pos[3]-(sensor_pos[6]+sensor_pos[7])/2
              rad=np.arcsin(err[0]/0.061)
              rad=threshold(rad,deg2rad(3))
        else:
           rad=0
        quat=tfs.euler.euler2quat(0,0,-rad*2-0.4281244919822008,"sxyz")
        desired_angle.append(rad)
        desired_quat.append(quat)
    model.geom_quat[25]=desired_quat[0]
    model.geom_quat[26]=desired_quat[0]
    model.geom_quat[27]=desired_quat[1]
    model.geom_quat[28]=desired_quat[1]
    model.geom_quat[49]=desired_quat[2]
    model.geom_quat[50]=desired_quat[2]
    model.geom_quat[51]=desired_quat[3]
    model.geom_quat[52]=desired_quat[3]

    return desired_angle,model

def CoP_compute(r_CoM,CoP_old,sensor_pos,sensor_data,rise):
    x_cop = 0
    y_cop = 0
    for idx,pos in enumerate(sensor_pos):
       sensor_pos[idx]=pos-r_CoM
    #DSP 
    if rise==0:
       for i in range(len(sensor_pos)):
            x_cop += sensor_data[i] * sensor_pos[i][0]
            y_cop += sensor_data[i] * sensor_pos[i][1]
    #SSP #Switch
    elif rise==1 or rise==3:#右腳懸空/l2r
       for i in range(int(len(sensor_pos)/2)):
            x_cop += sensor_data[i] * sensor_pos[i][0]
            y_cop += sensor_data[i] * sensor_pos[i][1]
    elif rise==2 or rise==4:#左腳懸空/r2l
       for i in range(int(len(sensor_pos)/2)):
            x_cop += sensor_data[i+4] * sensor_pos[i+4][0]
            y_cop += sensor_data[i+4] * sensor_pos[i+4][1]
    x_cop /= (sum(sensor_data))+0.000001
    y_cop /= (sum(sensor_data))+0.000001
    #做均值濾波
    x_cop = ((x_cop+r_CoM[0])+CoP_old[0][0]+CoP_old[1][0])/3
    y_cop = ((y_cop+r_CoM[1])+CoP_old[0][1]+CoP_old[1][1])/3

    return np.array([x_cop,y_cop])

def y_CoP_control(d_motion,rise,CoP_y,Rd_end_y,Ld_end_y,weight):
    theta=0
    #DSP, SP
    if rise==0 or rise==3 or rise==4:
       if CoP_y<Rd_end_y-theta: 
          d_motion[2]+= d_motion[2]*weight[0]
          d_motion[8]+=d_motion[8]*weight[0]
          d_motion[6]+= d_motion[6]*weight[1]
          d_motion[12]+=d_motion[12]*weight[1]
       elif CoP_y>Ld_end_y+theta: 
          d_motion[2]-= d_motion[2]*weight[0] 
          d_motion[8]-=d_motion[8]*weight[0]
          d_motion[6]-= d_motion[6]*weight[1] 
          d_motion[12]-=d_motion[12]*weight[1]
    #SSP
    elif rise==1:#右腳懸空
       if CoP_y>Ld_end_y+theta: 
          d_motion[2]-= d_motion[2]*weight[2]
          d_motion[6]-= d_motion[6]*weight[3] 

    elif rise==2:#左腳懸空
       if CoP_y<Rd_end_y-theta :
          d_motion[8]+= d_motion[8]*weight[2]
          d_motion[12]+= d_motion[12]*weight[3]

    return d_motion[2],d_motion[6],d_motion[8],d_motion[12]

def x_CoP_control(d_motion,rise,CoP_x,Rd_end_x,Ld_end_x,weight):
    theta=0.05
    #DSP 
    if rise==0:
       if CoP_x>max(Rd_end_x,Ld_end_x)+theta: 
          d_motion[3]= d_motion[3]*(1-weight[0])-d_motion[3]*weight[1]
          d_motion[4]= d_motion[4]*(1+weight[0])+d_motion[4]*weight[1]
          d_motion[5] = d_motion[5]*(1-weight[0])-d_motion[5]*weight[1]

          d_motion[9]= d_motion[9]*(1-weight[0])-d_motion[9]*weight[1]
          d_motion[10]= d_motion[10]*(1+weight[0])+d_motion[10]*weight[1]
          d_motion[11]= d_motion[11]*(1-weight[0])-d_motion[11]*weight[1]

       elif CoP_x<min(Rd_end_x,Ld_end_x)-theta : 
          d_motion[3]= d_motion[3]*(1+weight[0])+d_motion[3]*weight[1]
          d_motion[4]= d_motion[4]*(1-weight[0])-d_motion[4]*weight[1]
          d_motion[5] = d_motion[5]*(1+weight[0])+d_motion[5]*weight[1]

          d_motion[9]= d_motion[9]*(1+weight[0])+d_motion[9]*weight[1]
          d_motion[10]= d_motion[10]*(1-weight[0])-d_motion[10]*weight[1]
          d_motion[11]= d_motion[11]*(1+weight[0])+d_motion[11]*weight[1]
    #SSP
    elif rise==1:#右腳懸空
       if CoP_x>Ld_end_x+theta: 
          d_motion[3]= d_motion[3]*(1-weight[0])-d_motion[3]*weight[3]
          d_motion[4]= d_motion[4]*(1+weight[0])+d_motion[4]*weight[3]
          d_motion[5] = d_motion[5]*(1-weight[0])-d_motion[5]*weight[3]

       elif CoP_x<Ld_end_x-theta : 
          d_motion[3]= d_motion[3]*(1+weight[0])+d_motion[3]*weight[3]
          d_motion[4]= d_motion[4]*(1-weight[0])-d_motion[4]*weight[3]
          d_motion[5]= d_motion[5]*(1+weight[0])+d_motion[5]*weight[3]

    elif rise==2:#左腳懸空
       if CoP_x>Rd_end_x+theta: 
          d_motion[9]= d_motion[9]*(1-weight[0])-d_motion[9]*weight[3]
          d_motion[10]= d_motion[10]*(1+weight[0])+d_motion[10]*weight[3]
          d_motion[11]= d_motion[11]*(1-weight[0])-d_motion[11]*weight[3]

       elif CoP_x<Rd_end_x-theta: 
          d_motion[9]= d_motion[9]*(1+weight[0])+d_motion[9]*weight[3]
          d_motion[10]= d_motion[10]*(1-weight[0])-d_motion[10]*weight[3]
          d_motion[11]= d_motion[11]*(1+weight[0])+d_motion[11]*weight[3]
    #SP
    elif rise==3:#l2r
        if CoP_x>(Rd_end_x+Ld_end_x)/2 : 
            d_motion[3]= d_motion[3]*(1+weight[0])+d_motion[3]*weight[2]
            d_motion[4]= d_motion[4]*(1-weight[0])-d_motion[4]*weight[2]
            d_motion[5]= d_motion[5]*(1+weight[0])+d_motion[5]*weight[2]

            d_motion[9]= d_motion[9]*(1-weight[0])-d_motion[9]*weight[2]
            d_motion[10]= d_motion[10]*(1+weight[0])+d_motion[10]*weight[2]
            d_motion[11]= d_motion[11]*(1-weight[0])-d_motion[11]*weight[2]

    elif rise==4:#r2l
       if CoP_x>(Rd_end_x+Ld_end_x)/2 : 
            d_motion[9]= d_motion[9]*(1+weight[0])+d_motion[9]*weight[2]
            d_motion[10]= d_motion[10]*(1-weight[0])-d_motion[10]*weight[2]
            d_motion[11]= d_motion[11]*(1+weight[0])+d_motion[11]*weight[2]

            d_motion[3]= d_motion[3]*(1-weight[0])-d_motion[3]*weight[2]
            d_motion[4]= d_motion[4]*(1+weight[0])+d_motion[4]*weight[2]
            d_motion[5] = d_motion[5]*(1-weight[0])-d_motion[5]*weight[2]

    return d_motion[5],d_motion[11],d_motion[3],d_motion[4],d_motion[9],d_motion[10]

#################### Single Speed Transitions ####################
def Pattern_Change_SF(d_motion,rise,Rd_end,Ld_end,RR,LR,euler_angle,Cmd,CoP_ctrl,weight):
    L=[0.102,0.35795 ,0.36642 ,0.029 , 0.11175]
    Rd_end[1]=-(Rd_end[1]+0.105)
    Ld_end[1]=-(Ld_end[1]-0.105)
    pitch=euler_angle[1]
    range=3
    change=False

    if Cmd==[0,1] : #慢到快
        value_z=weight[0] 
        value_x=weight[0]
        value=weight[1]
    elif Cmd==[1,1] or Cmd==[1,3]: #快/快停
        value_z=weight[2]  
        value_x=weight[2]
        value=weight[3]
    else:  #慢到慢
        value_z=weight[4] 
        value_x=weight[4]
        value=weight[5]
    #value_z蹲為正/起為負
    #value_x蹲為負/起為正
    if pitch<-range:
        change=True
        if rise==1 or rise==3: #右腳離地/l2r
            PR=Rd_end+[0,0,-value_z]
            PL=Ld_end+[+value_x,0,-value_z]
        elif rise==2 or rise==4: #左腳離地/r2l
            PL=Rd_end+[0,0,-value_z]
            PR=Ld_end+[+value_x,0,-value_z]
        elif rise==0: 
            PR=Rd_end+[0,0,-value] 
            PL=Ld_end+[0,0,-value]
    elif pitch>range:
        change=True
        if rise==1 or rise==3: #右腳離地/l2r
            PR=Rd_end+[0,0,+value_z]
            PL=Ld_end+[-value_x,0,+value_z]
        elif rise==2 or rise==4: #左腳離地/r2l
            PL=Rd_end+[0,0,+value_z]
            PR=Ld_end+[-value_x,0,+value_z]
        elif rise==0: 
            PR=Rd_end+[0,0,+value] 
            PL=Ld_end+[0,0,+value]

    if change:
        d_motion.iloc[2,3:5]+=InvK(PL,RR,L,0)[2:4]-d_motion.iloc[2,3:5]
        d_motion.iloc[2,9:11]+=InvK(PR,LR,L,0)[2:4]-d_motion.iloc[2,9:11]
        d_motion.iloc[2,3:5]=0.3*d_motion.iloc[0,3:5]+0.3*d_motion.iloc[1,3:5]+0.4*d_motion.iloc[2,3:5]
        d_motion.iloc[2,9:11]=0.3*d_motion.iloc[0,9:11]+0.3*d_motion.iloc[1,9:11]+0.4*d_motion.iloc[2,9:11]  
        
    else:
        d_motion.iloc[2,3]=CoP_ctrl[0]
        d_motion.iloc[2,4]=CoP_ctrl[1]
        d_motion.iloc[2,9]=CoP_ctrl[2]
        d_motion.iloc[2,10]=CoP_ctrl[3]

    return d_motion.iloc[2,3],d_motion.iloc[2,4],d_motion.iloc[2,9],d_motion.iloc[2,10],change

def Pattern_Change_FS(d_motion,rise,Rd_end,Ld_end,RR,LR,euler_angle,Cmd,CoP_ctrl,weight):
    L=[0.102,0.35795 ,0.36642 ,0.029 , 0.11175]
    Rd_end[1]=-(Rd_end[1]+0.105)
    Ld_end[1]=-(Ld_end[1]-0.105)
    pitch=euler_angle[1]
    range=3
    change=False
    if Cmd==[1,2] :  #快到慢
        value_z=weight[0]
        value_x=weight[0]
        value=weight[1]
    elif Cmd==[0,2] or Cmd==[0,3]: #慢/慢停
        value_z=weight[2] 
        value_x=weight[2]
        value=weight[3]
    else:             #快到快
        value_z=weight[4]
        value_x=weight[4]
        value=weight[5]
    #value_z蹲為正/起為負
    #value_x蹲為負/起為正
    if pitch<-range:
        change=True
        if rise==1 or rise==3: #右腳離地/l2r
            PR=Rd_end+[0,0,-value_z]
            PL=Ld_end+[+value_x,0,-value_z]
        elif rise==2 or rise==4: #左腳離地/r2l
            PL=Rd_end+[0,0,-value_z]
            PR=Ld_end+[+value_x,0,-value_z]
        elif rise==0: 
            PR=Rd_end+[0,0,-value] 
            PL=Ld_end+[0,0,-value]
    elif pitch>range:
        change=True
        if rise==1 or rise==3: #右腳離地/l2r
            PR=Rd_end+[0,0,+value_z]
            PL=Ld_end+[-value_x,0,+value_z]
        elif rise==2 or rise==4: #左腳離地/r2l
            PL=Rd_end+[0,0,+value_z]
            PR=Ld_end+[-value_x,0,+value_z]
        elif rise==0: 
            PR=Rd_end+[0,0,+value] 
            PL=Ld_end+[0,0,+value]
    if change:
        d_motion.iloc[2,3:5]+=InvK(PL,RR,L,0)[2:4]-d_motion.iloc[2,3:5]
        d_motion.iloc[2,9:11]+=InvK(PR,LR,L,0)[2:4]-d_motion.iloc[2,9:11]
        d_motion.iloc[2,3:5]=0.3*d_motion.iloc[0,3:5]+0.3*d_motion.iloc[1,3:5]+0.4*d_motion.iloc[2,3:5]
        d_motion.iloc[2,9:11]=0.3*d_motion.iloc[0,9:11]+0.3*d_motion.iloc[1,9:11]+0.4*d_motion.iloc[2,9:11]  
    else:
        d_motion.iloc[2,3]=CoP_ctrl[0]
        d_motion.iloc[2,4]=CoP_ctrl[1]
        d_motion.iloc[2,9]=CoP_ctrl[2]
        d_motion.iloc[2,10]=CoP_ctrl[3]

    return d_motion.iloc[2,3],d_motion.iloc[2,4],d_motion.iloc[2,9],d_motion.iloc[2,10],change

#################### Dual Speed Transitions ####################
def Pattern_Change_Ctrl(d_motion,rise,Rd_end,Ld_end,RR,LR,euler_angle,Cmd,CoP_ctrl,weight): 
    L=[0.102,0.35795 ,0.36642 ,0.029 , 0.11175]
    Rd_end[1]=-(Rd_end[1]+0.105)
    Ld_end[1]=-(Ld_end[1]-0.105)
    pitch=euler_angle[1]
    range=3
    change=False

    if Cmd==[1,2] : 
        value_z=weight[0]
        value_x=weight[0]
        value=weight[1]
    elif Cmd==[0,2] or Cmd==[0,3]: 
        value_z=weight[2] 
        value_x=weight[2]
        value=weight[3]
    elif Cmd==[1,0] :   
        value_z=weight[4]
        value_x=weight[4]
        value=weight[5]
    elif Cmd==[0,1] :
        value_z=weight[6]
        value_x=weight[6]
        value=weight[7]
    elif Cmd==[1,1] or Cmd==[1,3] :
        value_z=weight[8]
        value_x=weight[8]
        value=weight[9]
    elif Cmd==[0,0] :
        value_z=weight[10]
        value_x=weight[10]
        value=weight[11]
    else :
        print("out of control")
    #value_z蹲為正/起為負
    #value_x蹲為負/起為正
    if pitch<-range:
        change=True
        if rise==1 or rise==3: #右腳離地/l2r
            PR=Rd_end+[0,0,-value_z]
            PL=Ld_end+[+value_x,0,-value_z]
        elif rise==2 or rise==4: #左腳離地/r2l
            PL=Rd_end+[0,0,-value_z]
            PR=Ld_end+[+value_x,0,-value_z]
        elif rise==0: 
            PR=Rd_end+[0,0,-value] 
            PL=Ld_end+[0,0,-value]
    elif pitch>range:
        change=True
        if rise==1 or rise==3: #右腳離地/l2r
            PR=Rd_end+[0,0,+value_z]
            PL=Ld_end+[-value_x,0,+value_z]
        elif rise==2 or rise==4: #左腳離地/r2l
            PL=Rd_end+[0,0,+value_z]
            PR=Ld_end+[-value_x,0,+value_z]
        elif rise==0: 
            PR=Rd_end+[0,0,+value] 
            PL=Ld_end+[0,0,+value]
    if change:
        d_motion.iloc[2,3:5]+=InvK(PL,RR,L,0)[2:4]-d_motion.iloc[2,3:5]
        d_motion.iloc[2,9:11]+=InvK(PR,LR,L,0)[2:4]-d_motion.iloc[2,9:11]
        d_motion.iloc[2,3:5]=0.3*d_motion.iloc[0,3:5]+0.3*d_motion.iloc[1,3:5]+0.4*d_motion.iloc[2,3:5]
        d_motion.iloc[2,9:11]=0.3*d_motion.iloc[0,9:11]+0.3*d_motion.iloc[1,9:11]+0.4*d_motion.iloc[2,9:11]  
        
    else:
        d_motion.iloc[2,3]=CoP_ctrl[0]
        d_motion.iloc[2,4]=CoP_ctrl[1]
        d_motion.iloc[2,9]=CoP_ctrl[2]
        d_motion.iloc[2,10]=CoP_ctrl[3]

    return d_motion.iloc[2,3],d_motion.iloc[2,4],d_motion.iloc[2,9],d_motion.iloc[2,10],change


