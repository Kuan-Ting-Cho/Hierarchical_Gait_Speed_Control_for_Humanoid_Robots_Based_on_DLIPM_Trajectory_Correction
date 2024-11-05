import numpy as np
import pandas as pd
from LIPM import *
from Simulate.Plot import *
import time
# 機器人估重 19.74 Kg
def deg2rad(angle):
    return angle*np.pi/180
# 將弧度換算角度進行控制運算
def rad2deg(radius):
    return radius/np.pi*180    
# 將Motion_data做線性插值，回傳插值動作
def Linear_interp(motion,index1,index2,num):
    motion1=motion.loc[index1].tolist()
    motion2=motion.loc[index2].tolist()
    motion_interp = np.linspace(motion1, motion2, num)

    return motion_interp
def initial_pos():
    # 以下給角度 (單位：degree) L3 = L8, L9 = L_ankle_pitch, R3 = R8, R9 = -R_ankle_pitch
    init_pos = [         #              qpos xpos(,1 base)            
        0, 0, 1.032,    # 機器人base起始位置 (不用動) 
        1, 0, 0, 0,      # 機器人起始四元數 (不用動)
        0,               # trunk         ,7  ,2
        0,               # L_hip_yaw     ,8  ,3
        0,               # L_hip_roll    ,9  ,4
        0,               # L_hip_pitch   ,10 ,5
        0,               # L3            ,11 ,6
        0,               # L4            ,12 ,7
        0,               # L8            ,13 ,8
        0,               # L_ankle_pitch ,14 ,9
        0,               # L_ankle_roll  ,15 ,10
        # 0,             # L_f_front         ,11
        # 0,             # L_f_back          ,12
        0,               # L9            ,16 ,13
        0,               # L10           ,17 ,14
        0,               # R_hip_yaw     ,18 ,15
        0,               # R_hip_roll    ,19 ,16
        0,               # R_hip_pitch   ,20 ,17
        0,               # R3            ,21 ,18
        0,               # R4            ,22 ,19
        0,               # R8            ,23 ,20
        0,               # R_ankle_pitch ,24 ,21
        0,               # R_ankle_roll  ,25 ,22
        # 0,             # R_f_front         ,23
        # 0,             # R_f_back          ,24
        0,               # R9            ,26 ,25
        0,               # R10           ,27 ,26
    ]
    init_pos[7:] = list(deg2rad(np.array(init_pos[7:])))
    # 以下給想要控制到的角度 (單位：degree)
    controller = [
        0,          # trunck
        0,          # L_hip_yaw
        0,          # L_hip_roll
        0,          # L_hip_pitch
        0,          # L8 
        0,          # L_ankle_pitch
        0,          # L_ankle_roll
        0,          # R_hip_yaw
        0,          # R_hip_roll
        0,          # R_hip_pitch
        0,          # R8
        0,          # R_ankle_pitch 
        0,          # R_ankle_roll
    ]
    acc_err = [0]*len(controller)
    return init_pos,acc_err
# 將Motion_data做線性插值，回傳加入插值動作的新Motion_data
def Data_preprocess(motion,balance_step,Rd_end,Ld_end,d_CoM,d_CoMv):
    Rd_end=pd.DataFrame(Rd_end).T
    Ld_end=pd.DataFrame(Ld_end).T
    d_CoM=pd.DataFrame(d_CoM).T
    d_CoMv=pd.DataFrame(d_CoMv).T 
    # 左右腳對調
    motion_left=motion.iloc[:,6:]
    motion_right=motion.iloc[:,0:6]
    motion=pd.concat([motion_left,motion_right],axis=1,ignore_index=True)
    # 加一全為零的新欄(for trunk)
    df = pd.DataFrame([deg2rad(0)]*len(motion.iloc[:, 0]))
    motion_ori=pd.concat([df,motion],axis=1,ignore_index=True)
    # 加一全為零的新列(站直)
    df1 = pd.DataFrame([0.0]*len(motion_ori.loc[0])).T
    motion=pd.concat([df1,motion_ori],axis=0,ignore_index=True)
    d_CoM1=pd.DataFrame([0.0,0.0,1.032]).T
    d_CoM=pd.concat([d_CoM1,d_CoM],axis=0,ignore_index=True)
    d_CoMv1=pd.DataFrame([0.0,0.0,0.0]).T
    d_CoMv=pd.concat([d_CoMv1,d_CoMv],axis=0,ignore_index=True)
    Rd_end1=pd.DataFrame([0.0,-0.105,-1.032]).T
    Rd_end=pd.concat([Rd_end1,Rd_end],axis=0,ignore_index=True)
    Ld_end1=pd.DataFrame([0.0,0.105,-1.032]).T
    Ld_end=pd.concat([Ld_end1,Ld_end],axis=0,ignore_index=True)
    # 在站直與初始蹲姿間做插值
    df2 = pd.DataFrame(Linear_interp(motion,0,1,balance_step))
    motion=pd.concat([df2,motion],axis=0,ignore_index=True)
    motion = motion.drop(balance_step).reset_index(drop=True)
    d_CoM2=pd.DataFrame(Linear_interp(d_CoM,0,1,balance_step))
    d_CoM=pd.concat([d_CoM2,d_CoM],axis=0,ignore_index=True)
    d_CoM = d_CoM.drop(balance_step).reset_index(drop=True)
    d_CoMv2=pd.DataFrame(Linear_interp(d_CoMv,0,0,balance_step))
    d_CoMv=pd.concat([d_CoMv2,d_CoMv],axis=0,ignore_index=True)
    d_CoMv = d_CoMv.drop(balance_step).reset_index(drop=True)
    Rd_end2=pd.DataFrame(Linear_interp(Rd_end,0,1,balance_step))
    Rd_end=pd.concat([Rd_end2,Rd_end],axis=0,ignore_index=True)
    Rd_end = Rd_end.drop(balance_step).reset_index(drop=True)
    Ld_end2=pd.DataFrame(Linear_interp(Ld_end,0,1,balance_step))
    Ld_end=pd.concat([Ld_end2,Ld_end],axis=0,ignore_index=True)
    Ld_end = Ld_end.drop(balance_step).reset_index(drop=True)
    #加入走完兩步平衡所需的Step(這邊給300 time steps)
    df3 = pd.DataFrame(Linear_interp(motion,len(motion.iloc[:,0])-1,len(motion.iloc[:,0])-1,300))
    motion=pd.concat([motion,df3],axis=0,ignore_index=True)
    d_CoM3 = pd.DataFrame(Linear_interp(d_CoM,len(d_CoM.iloc[:,0])-1,len(d_CoM.iloc[:,0])-1,300))
    d_CoM=pd.concat([d_CoM,d_CoM3],axis=0,ignore_index=True)
    d_CoMv3=pd.DataFrame(Linear_interp(d_CoMv,0,0,300))
    d_CoMv=pd.concat([d_CoMv,d_CoMv3],axis=0,ignore_index=True)
    Rd_end3 = pd.DataFrame(Linear_interp(Rd_end,len(Rd_end.iloc[:,0])-1,len(Rd_end.iloc[:,0])-1,300))
    Rd_end=pd.concat([Rd_end,Rd_end3],axis=0,ignore_index=True)
    Ld_end3 = pd.DataFrame(Linear_interp(Ld_end,len(Ld_end.iloc[:,0])-1,len(Ld_end.iloc[:,0])-1,300))
    Ld_end=pd.concat([Ld_end,Ld_end3],axis=0,ignore_index=True)
    return motion,Rd_end,Ld_end,d_CoM,d_CoMv


def stop_data(motion,balance_step,Rd_end,Ld_end,d_CoM,d_CoMv,one_step_t,stop): #stop 1:right 2:left
    one_step=int(one_step_t/0.01)
    Rd_end=pd.DataFrame(Rd_end).T
    Ld_end=pd.DataFrame(Ld_end).T
    d_CoM=pd.DataFrame(d_CoM).T
    d_CoMv=pd.DataFrame(d_CoMv).T 
    # 左右腳對調
    motion_left=motion.iloc[:,6:]
    motion_right=motion.iloc[:,0:6]
    motion=pd.concat([motion_left,motion_right],axis=1,ignore_index=True)
    # 加一全為零的新欄(for trunk)
    df = pd.DataFrame([deg2rad(0)]*len(motion.iloc[:, 0]))
    motion_ori=pd.concat([df,motion],axis=1,ignore_index=True)
    # 加一全為零的新列(站直)
    df1 = pd.DataFrame([0.0]*len(motion_ori.loc[0])).T
    motion=pd.concat([df1,motion_ori],axis=0,ignore_index=True)
    d_CoM1=pd.DataFrame([0.0,0.0,1.032]).T
    d_CoM=pd.concat([d_CoM1,d_CoM],axis=0,ignore_index=True)
    d_CoMv1=pd.DataFrame([0.0,0.0,0.0]).T
    d_CoMv=pd.concat([d_CoMv1,d_CoMv],axis=0,ignore_index=True)
    Rd_end1=pd.DataFrame([0.0,-0.105,-1.032]).T
    Rd_end=pd.concat([Rd_end1,Rd_end],axis=0,ignore_index=True)
    Ld_end1=pd.DataFrame([0.0,0.105,-1.032]).T
    Ld_end=pd.concat([Ld_end1,Ld_end],axis=0,ignore_index=True)
    # 在站直與初始蹲姿間做插值
    df2 = pd.DataFrame(Linear_interp(motion,0,1,balance_step))
    motion=pd.concat([df2,motion],axis=0,ignore_index=True)
    motion = motion.drop(balance_step).reset_index(drop=True)
    d_CoM2=pd.DataFrame(Linear_interp(d_CoM,0,1,balance_step))
    d_CoM=pd.concat([d_CoM2,d_CoM],axis=0,ignore_index=True)
    d_CoM = d_CoM.drop(balance_step).reset_index(drop=True)
    d_CoMv2=pd.DataFrame(Linear_interp(d_CoMv,0,0,balance_step))
    d_CoMv=pd.concat([d_CoMv2,d_CoMv],axis=0,ignore_index=True)
    d_CoMv = d_CoMv.drop(balance_step).reset_index(drop=True)
    Rd_end2=pd.DataFrame(Linear_interp(Rd_end,0,1,balance_step))
    Rd_end=pd.concat([Rd_end2,Rd_end],axis=0,ignore_index=True)
    Rd_end = Rd_end.drop(balance_step).reset_index(drop=True)
    Ld_end2=pd.DataFrame(Linear_interp(Ld_end,0,1,balance_step))
    Ld_end=pd.concat([Ld_end2,Ld_end],axis=0,ignore_index=True)
    Ld_end = Ld_end.drop(balance_step).reset_index(drop=True)
    if stop==1: #right
        #加入走完兩步平衡所需的Step(這邊給300 time steps)
        df3 = pd.DataFrame(Linear_interp(motion,len(motion.iloc[:,0])-1-one_step,len(motion.iloc[:,0])-1-one_step,300))
        motion=pd.concat([motion[0:int(len(motion.iloc[:,0])-one_step)],df3],axis=0,ignore_index=True)
        d_CoM3 = pd.DataFrame(Linear_interp(d_CoM,len(d_CoM.iloc[:,0])-1-one_step,len(d_CoM.iloc[:,0])-1-one_step,300))
        d_CoM=pd.concat([d_CoM[0:int(len(d_CoM.iloc[:,0])-one_step)],d_CoM3],axis=0,ignore_index=True)
        d_CoMv3=pd.DataFrame(Linear_interp(d_CoMv,0,0,300))
        d_CoMv=pd.concat([d_CoMv[0:int(len(d_CoMv.iloc[:,0])-one_step)],d_CoMv3],axis=0,ignore_index=True)
        Rd_end3 = pd.DataFrame(Linear_interp(Rd_end,len(Rd_end.iloc[:,0])-1-one_step,len(Rd_end.iloc[:,0])-1-one_step,300))
        Rd_end=pd.concat([Rd_end[0:int(len(Rd_end.iloc[:,0])-one_step)],Rd_end3],axis=0,ignore_index=True)
        Ld_end3 = pd.DataFrame(Linear_interp(Ld_end,len(Ld_end.iloc[:,0])-1-one_step,len(Ld_end.iloc[:,0])-1-one_step,300))
        Ld_end=pd.concat([Ld_end[0:int(len(Ld_end.iloc[:,0])-one_step)],Ld_end3],axis=0,ignore_index=True)

    elif stop==2:
        #加入走完兩步平衡所需的Step(這邊給300 time steps)
        df3 = pd.DataFrame(Linear_interp(motion,len(motion.iloc[:,0])-1-one_step*0,len(motion.iloc[:,0])-1-one_step*0,300))
        motion=pd.concat([motion[0:int(len(motion.iloc[:,0])-one_step*1)],motion[int(len(motion.iloc[:,0])-one_step*1):],df3],axis=0,ignore_index=True)
        d_CoM3 = pd.DataFrame(Linear_interp(d_CoM,len(d_CoM.iloc[:,0])-1-one_step*0,len(d_CoM.iloc[:,0])-1-one_step*0,300))
        d_CoM=pd.concat([d_CoM[0:int(len(d_CoM.iloc[:,0])-one_step*1)],d_CoM[int(len(d_CoM.iloc[:,0])-one_step*1):],d_CoM3],axis=0,ignore_index=True)
        d_CoMv3=pd.DataFrame(Linear_interp(d_CoMv,0,0,300))
        d_CoMv=pd.concat([d_CoMv[0:int(len(d_CoMv.iloc[:,0])-one_step*1)],d_CoMv[int(len(d_CoMv.iloc[:,0])-one_step*1):],d_CoMv3],axis=0,ignore_index=True)
        Rd_end3 = pd.DataFrame(Linear_interp(Rd_end,len(Rd_end.iloc[:,0])-1-one_step*0,len(Rd_end.iloc[:,0])-1-one_step*0,300))
        Rd_end=pd.concat([Rd_end[0:int(len(Rd_end.iloc[:,0])-one_step*1)],Rd_end[int(len(Rd_end.iloc[:,0])-one_step*1):],Rd_end3],axis=0,ignore_index=True)
        Ld_end3 = pd.DataFrame(Linear_interp(Ld_end,len(Ld_end.iloc[:,0])-1-one_step*0,len(Ld_end.iloc[:,0])-1-one_step*0,300))
        Ld_end=pd.concat([Ld_end[0:int(len(Ld_end.iloc[:,0])-one_step*1)],Ld_end[int(len(Ld_end.iloc[:,0])-one_step*1):],Ld_end3],axis=0,ignore_index=True)
    return motion,Rd_end,Ld_end,d_CoM,d_CoMv

def evaluation_sf(R_end_err,L_end_err,CoM_err,CoMv_err,i,mode,changed_step,one_step_time):
    slow_step=changed_step[0]
    R_ERR=[0.0,0.0,0.0]
    L_ERR=[0.0,0.0,0.0]
    CoM_ERR=[0.0,0.0,0.0]
    CoMv_ERR=[0.0,0.0,0.0]
    s_step=int(one_step_time[0]/0.01) 
    f_step=int(one_step_time[1]/0.01) 
    bias=abs(one_step_time[0]-one_step_time[1])
    total_step=16
    balence_step=0
    if i>(49+slow_step*s_step+f_step*(total_step-slow_step)): #超過16步，算平衡時間
        balence_step=i-((50+slow_step*s_step+f_step*(total_step-slow_step)))
        i=(49+slow_step*s_step+f_step*(total_step-slow_step))

    if i<(50+slow_step*s_step):
        step=int((i-49)/s_step)
        if int((i-49)/s_step)!=0:
            R_ERR=sum(R_end_err)/step
            L_ERR=sum(L_end_err)/step
            CoM_ERR=sum(CoM_err)/step
            CoMv_ERR=sum(CoMv_err)/step
            fitness=(16-step)*np.exp(2*bias+1-(step/slow_step))+abs(R_ERR[1])+abs(L_ERR[1])+abs(CoM_ERR[1])+np.exp((1-balence_step/300)*((2*one_step_time[1])**2))
        else:
            fitness=20 #一步都沒完成懲罰值
    else:
        R_ERR=sum(R_end_err[0:50+slow_step*s_step])/int(slow_step)
        L_ERR=sum(L_end_err[0:50+slow_step*s_step])/int(slow_step)
        CoM_ERR=sum(CoM_err[0:50+slow_step*s_step])/int(slow_step)
        CoMv_ERR=sum(CoMv_err[0:50+slow_step*s_step])/int(slow_step)
        if int((i-(49+slow_step*s_step))/f_step)!=0:
            R_ERR+=sum(R_end_err[50+slow_step*s_step:])/int((i-(49+slow_step*s_step))/f_step)
            L_ERR+=sum(L_end_err[50+slow_step*s_step:])/int((i-(49+slow_step*s_step))/f_step)
            CoM_ERR+=sum(CoM_err[50+slow_step*s_step:])/int((i-(49+slow_step*s_step))/f_step)
            CoMv_ERR+=sum(CoMv_err[50+slow_step*s_step:])/int((i-(49+slow_step*s_step))/f_step)
        step=int((i-(49+slow_step*s_step))/f_step+slow_step)
        fitness=(16-step)*np.exp(2*bias)+abs(R_ERR[1])+abs(L_ERR[1])+abs(CoM_ERR[1])+np.exp((1-balence_step/300)*((2*one_step_time[1])**2))
    print("one step time: [%1.1f,%1.1f], simulate_step: %4.1f, foot_step: %2.1f, balence_step: %4.1f, fitness:  %2.5f" %(one_step_time[0],one_step_time[1],i,step,balence_step,fitness))
    return fitness,balence_step

def evaluation_fs(R_end_err,L_end_err,CoM_err,CoMv_err,i,mode,changed_step,one_step_time):
    fast_step=changed_step[0]
    R_ERR=[0.0,0.0,0.0]
    L_ERR=[0.0,0.0,0.0]
    CoM_ERR=[0.0,0.0,0.0]
    CoMv_ERR=[0.0,0.0,0.0]
    s_step=int(one_step_time[0]/0.01) 
    f_step=int(one_step_time[1]/0.01)
    bias=abs(one_step_time[0]-one_step_time[1])
    total_step=16
    balence_step=0
    if i>(49+fast_step*f_step+s_step*(total_step-fast_step)): #超過16步，算平衡時間
        balence_step=i-(50+fast_step*f_step+s_step*(total_step-fast_step))
        i=49+fast_step*f_step+s_step*(total_step-fast_step)

    if i<(50+fast_step*f_step):
        step=int((i-49)/f_step)
        if int((i-49)/f_step)!=0:
            R_ERR=sum(R_end_err)/step
            L_ERR=sum(L_end_err)/step
            CoM_ERR=sum(CoM_err)/step
            CoMv_ERR=sum(CoMv_err)/step
            fitness=(16-step)*np.exp(2*bias+1-(step/fast_step))+abs(R_ERR[1])+abs(L_ERR[1])+abs(CoM_ERR[1])+np.exp((1-balence_step/300)*((2*one_step_time[0])**2))
        else:
            fitness=20 
    else:

        R_ERR=sum(R_end_err[0:50+fast_step*f_step])/int(fast_step)
        L_ERR=sum(L_end_err[0:50+fast_step*f_step])/int(fast_step)
        CoM_ERR=sum(CoM_err[0:50+fast_step*f_step])/int(fast_step)
        CoMv_ERR=sum(CoMv_err[0:50+fast_step*f_step])/int(fast_step)
        if int((i-(49+fast_step*f_step))/s_step)!=0:
            R_ERR+=sum(R_end_err[50+fast_step*f_step:])/int((i-(49+fast_step*f_step))/s_step)
            L_ERR+=sum(L_end_err[50+fast_step*f_step:])/int((i-(49+fast_step*f_step))/s_step)
            CoM_ERR+=sum(CoM_err[50+fast_step*f_step:])/int((i-(49+fast_step*f_step))/s_step)
            CoMv_ERR+=sum(CoMv_err[50+fast_step*f_step:])/int((i-(49+fast_step*f_step))/s_step)
        step=int((i-(49+fast_step*f_step))/s_step+fast_step)
        fitness=(16-step)*np.exp(2*bias)+abs(R_ERR[1])+abs(L_ERR[1])+abs(CoM_ERR[1])+np.exp((1-balence_step/300)*((2*one_step_time[0])**2))
    print("one step time: [%1.1f,%1.1f], simulate_step: %4.1f, foot_step: %2.1f, balence_step: %4.1f, fitness:  %2.5f" %(one_step_time[0],one_step_time[1],i,step,balence_step,fitness))
    return fitness,balence_step

def evaluation_n(R_end_err,L_end_err,CoM_err,CoMv_err,i,mode,changed_step,one_step_time):
    R_ERR=[0.0,0.0,0.0]
    L_ERR=[0.0,0.0,0.0]
    CoM_ERR=[0.0,0.0,0.0]
    total_step=20
    balence_step=0
    s_step=int(one_step_time[0]/0.01)
    f_step=int(one_step_time[1]/0.01)
    if mode==5:
        if i>(50+s_step*total_step): #超過20步，算平衡時間
            balence_step=i-(50+s_step*total_step)
            i=50+s_step*total_step
        step=int((i-50)/s_step)


    if step!=0:
        R_ERR=sum(R_end_err)/step
        L_ERR=sum(L_end_err)/step
        CoM_ERR=sum(CoM_err)/step
        CoMv_ERR=sum(CoMv_err)/step
        fitness=np.exp((1-step/20)*((2*one_step_time[0])**2))+abs(R_ERR[1])+abs(L_ERR[1])+abs(CoM_ERR[1])+np.exp((1-balence_step/300)*((2*one_step_time[0])**2))
    else:
        fitness=20
    print("one step time:  %1.1f, simulate_step: %4.1f, foot_step: %2.1f, balence_step: %4.1f, fitness:  %2.5f" %(one_step_time[0],i,step,balence_step,fitness))
    # print("i",i)
    # print("one step time",one_step_time[0])
    # print("foot_step",step)
    # print("R_ERR[1]",abs(R_ERR[1]))
    # print("L_ERR[1]",abs(L_ERR[1]))
    # print("CoM_ERR[1]",abs(CoM_ERR[1]))
    # print("balence_step",balence_step)
    # print("fitness",fitness)
    return fitness,balence_step

def Motion_Concat(Cmd,data,motion,d_motion,Rd_end,Ld_end,d_CoM,d_CoMv,RR,LR,Rd_end_w,Ld_end_w,one_step_t,step,d_CoMx,foot):
    #Cmd=[pre_Cmd,Cmd] #foot:1/right 2/left
    one_step=int(one_step_t/0.01)
    if Cmd[1]==3: #Stop
        if Cmd[0]==0:#Slow
            if foot==1:
                print("右腳停")
                d_motion2, Rd_end2, Ld_end2, d_CoM2, d_CoMv2=stop_data(motion,50,data.Rd_end,data.Ld_end,data.d_CoM,data.d_CoMv,one_step_t,1) #右腳停
                d_CoM2[d_CoM2.columns[0]]+=(d_CoMx-d_CoM2.iloc[50+6*one_step,0])
                d_motion = pd.concat([d_motion[0:step+1],d_motion2[50+6*one_step:]],axis=0,ignore_index=True)
                Rd_end = pd.concat([Rd_end[0:step+1],Rd_end2[50+6*one_step:]],axis=0,ignore_index=True)
                Ld_end = pd.concat([Ld_end[0:step+1],Ld_end2[50+6*one_step:]],axis=0,ignore_index=True)
                d_CoM = pd.concat([d_CoM[0:step+1],d_CoM2[50+6*one_step:]],axis=0,ignore_index=True)
                d_CoMv = pd.concat([d_CoMv[0:step+1],d_CoMv2[50+6*one_step:]],axis=0,ignore_index=True)
                RR = np.concatenate((RR[0:step+1, :],data.RR[50+6*one_step:, :]), axis=0)
                LR = np.concatenate((LR[0:step+1, :],data.LR[50+6*one_step:, :]), axis=0)
                Rd_end_w=pd.concat([Rd_end_w[0:step+1],Rd_end2[50+6*one_step:]+d_CoM2[50+6*one_step:]],axis=0,ignore_index=True)
                Ld_end_w=pd.concat([Ld_end_w[0:step+1],Ld_end2[50+6*one_step:]+d_CoM2[50+6*one_step:]],axis=0,ignore_index=True)
            if foot==2:
                print("#左腳停")
                d_motion2, Rd_end2, Ld_end2, d_CoM2, d_CoMv2=stop_data(motion,50,data.Rd_end,data.Ld_end,data.d_CoM,data.d_CoMv,one_step_t,2) #左腳停
                d_CoM2[d_CoM2.columns[0]]+=(d_CoMx-d_CoM2.iloc[50+7*one_step,0])
                d_motion = pd.concat([d_motion[0:step+1],d_motion2[50+7*one_step:]],axis=0,ignore_index=True)
                Rd_end = pd.concat([Rd_end[0:step+1],Rd_end2[50+7*one_step:]],axis=0,ignore_index=True)
                Ld_end = pd.concat([Ld_end[0:step+1],Ld_end2[50+7*one_step:]],axis=0,ignore_index=True)
                d_CoM = pd.concat([d_CoM[0:step+1],d_CoM2[50+7*one_step:]],axis=0,ignore_index=True)
                d_CoMv = pd.concat([d_CoMv[0:step+1],d_CoMv2[50+7*one_step:]],axis=0,ignore_index=True)
                RR = np.concatenate((RR[0:step+1, :],data.RR[50+7*one_step:, :]), axis=0)
                LR = np.concatenate((LR[0:step+1, :],data.LR[50+7*one_step:, :]), axis=0)
                Rd_end_w=pd.concat([Rd_end_w[0:step+1],Rd_end2[50+7*one_step:]+d_CoM2[50+7*one_step:]],axis=0,ignore_index=True)
                Ld_end_w=pd.concat([Ld_end_w[0:step+1],Ld_end2[50+7*one_step:]+d_CoM2[50+7*one_step:]],axis=0,ignore_index=True)
        elif Cmd[0]==1:#Fast
            if foot==1:
                print("右腳停")
                d_motion2, Rd_end2, Ld_end2, d_CoM2, d_CoMv2=stop_data(motion,50,data.Rd_end,data.Ld_end,data.d_CoM,data.d_CoMv,one_step_t,1) #右腳停
                d_CoM2[d_CoM2.columns[0]]+=(d_CoMx-d_CoM2.iloc[50+6*one_step,0])
                d_motion = pd.concat([d_motion[0:step+1],d_motion2[50+6*one_step:]],axis=0,ignore_index=True)
                Rd_end = pd.concat([Rd_end[0:step+1],Rd_end2[50+6*one_step:]],axis=0,ignore_index=True)
                Ld_end = pd.concat([Ld_end[0:step+1],Ld_end2[50+6*one_step:]],axis=0,ignore_index=True)
                d_CoM = pd.concat([d_CoM[0:step+1],d_CoM2[50+6*one_step:]],axis=0,ignore_index=True)
                d_CoMv = pd.concat([d_CoMv[0:step+1],d_CoMv2[50+6*one_step:]],axis=0,ignore_index=True)
                RR = np.concatenate((RR[0:step+1, :],data.RR[50+6*one_step:, :]), axis=0)
                LR = np.concatenate((LR[0:step+1, :],data.LR[50+6*one_step:, :]), axis=0)
                Rd_end_w=pd.concat([Rd_end_w[0:step+1],Rd_end2[50+6*one_step:]+d_CoM2[50+6*one_step:]],axis=0,ignore_index=True)
                Ld_end_w=pd.concat([Ld_end_w[0:step+1],Ld_end2[50+6*one_step:]+d_CoM2[50+6*one_step:]],axis=0,ignore_index=True)
            if foot==2: #左腳停
                print("#左腳停")
                d_motion2, Rd_end2, Ld_end2, d_CoM2, d_CoMv2=stop_data(motion,50,data.Rd_end,data.Ld_end,data.d_CoM,data.d_CoMv,one_step_t,2) #左腳停
                d_CoM2[d_CoM2.columns[0]]+=(d_CoMx-d_CoM2.iloc[50+7*one_step,0])
                d_motion = pd.concat([d_motion[0:step+1],d_motion2[50+7*one_step:]],axis=0,ignore_index=True)
                Rd_end = pd.concat([Rd_end[0:step+1],Rd_end2[50+7*one_step:]],axis=0,ignore_index=True)
                Ld_end = pd.concat([Ld_end[0:step+1],Ld_end2[50+7*one_step:]],axis=0,ignore_index=True)
                d_CoM = pd.concat([d_CoM[0:step+1],d_CoM2[50+7*one_step:]],axis=0,ignore_index=True)
                d_CoMv = pd.concat([d_CoMv[0:step+1],d_CoMv2[50+7*one_step:]],axis=0,ignore_index=True)
                RR = np.concatenate((RR[0:step+1, :],data.RR[50+7*one_step:, :]), axis=0)
                LR = np.concatenate((LR[0:step+1, :],data.LR[50+7*one_step:, :]), axis=0)
                Rd_end_w=pd.concat([Rd_end_w[0:step+1],Rd_end2[50+7*one_step:]+d_CoM2[50+7*one_step:]],axis=0,ignore_index=True)
                Ld_end_w=pd.concat([Ld_end_w[0:step+1],Ld_end2[50+7*one_step:]+d_CoM2[50+7*one_step:]],axis=0,ignore_index=True)
    elif Cmd[1]==1: #SF 
        d_motion2, Rd_end2, Ld_end2, d_CoM2, d_CoMv2=Data_preprocess(motion,50,data.Rd_end,data.Ld_end,data.d_CoM,data.d_CoMv)
        d_CoM2[d_CoM2.columns[0]]+=(d_CoMx-d_CoM2.iloc[50+2*one_step,0])
        d_motion = pd.concat([d_motion[0:step+1],d_motion2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        Rd_end = pd.concat([Rd_end[0:step+1],Rd_end2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        Ld_end = pd.concat([Ld_end[0:step+1],Ld_end2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        d_CoM = pd.concat([d_CoM[0:step+1],d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        d_CoMv = pd.concat([d_CoMv[0:step+1],d_CoMv2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        RR = np.concatenate((RR[0:step+1, :],data.RR[50+2*one_step:50+4*one_step, :]), axis=0)
        LR = np.concatenate((LR[0:step+1, :],data.LR[50+2*one_step:50+4*one_step, :]), axis=0)
        Rd_end_w=pd.concat([Rd_end_w[0:step+1],Rd_end2[50+2*one_step:50+4*one_step]+d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        Ld_end_w=pd.concat([Ld_end_w[0:step+1],Ld_end2[50+2*one_step:50+4*one_step]+d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
    elif Cmd[1]==2: #FS 
        d_motion2, Rd_end2, Ld_end2, d_CoM2, d_CoMv2=Data_preprocess(motion,50,data.Rd_end,data.Ld_end,data.d_CoM,data.d_CoMv)
        d_CoM2[d_CoM2.columns[0]]+=(d_CoMx-d_CoM2.iloc[50+2*one_step,0])
        d_motion = pd.concat([d_motion[0:step+1],d_motion2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        Rd_end = pd.concat([Rd_end[0:step+1],Rd_end2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        Ld_end = pd.concat([Ld_end[0:step+1],Ld_end2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        d_CoM = pd.concat([d_CoM[0:step+1],d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        d_CoMv = pd.concat([d_CoMv[0:step+1],d_CoMv2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        RR = np.concatenate((RR[0:step+1, :],data.RR[50+2*one_step:50+4*one_step, :]), axis=0)
        LR = np.concatenate((LR[0:step+1, :],data.LR[50+2*one_step:50+4*one_step, :]), axis=0)
        Rd_end_w=pd.concat([Rd_end_w[0:step+1],Rd_end2[50+2*one_step:50+4*one_step]+d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        Ld_end_w=pd.concat([Ld_end_w[0:step+1],Ld_end2[50+2*one_step:50+4*one_step]+d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
    else: #N
        if Cmd[0]==0:#Slow
            d_motion2, Rd_end2, Ld_end2, d_CoM2, d_CoMv2=Data_preprocess(motion,50,data.Rd_end,data.Ld_end,data.d_CoM,data.d_CoMv)
            d_CoM2[d_CoM2.columns[0]]+=(d_CoMx-d_CoM2.iloc[50+2*one_step,0])
            d_motion = pd.concat([d_motion[0:step+1],d_motion2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            Rd_end = pd.concat([Rd_end[0:step+1],Rd_end2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            Ld_end = pd.concat([Ld_end[0:step+1],Ld_end2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            d_CoM = pd.concat([d_CoM[0:step+1],d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            d_CoMv = pd.concat([d_CoMv[0:step+1],d_CoMv2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            RR = np.concatenate((RR[0:step+1, :],data.RR[50+2*one_step:50+4*one_step, :]), axis=0)
            LR = np.concatenate((LR[0:step+1, :],data.LR[50+2*one_step:50+4*one_step, :]), axis=0)
            Rd_end_w=pd.concat([Rd_end_w[0:step+1],Rd_end2[50+2*one_step:50+4*one_step]+d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            Ld_end_w=pd.concat([Ld_end_w[0:step+1],Ld_end2[50+2*one_step:50+4*one_step]+d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
        elif Cmd[0]==1:#Fast
            d_motion2, Rd_end2, Ld_end2, d_CoM2, d_CoMv2=Data_preprocess(motion,50,data.Rd_end,data.Ld_end,data.d_CoM,data.d_CoMv)
            d_CoM2[d_CoM2.columns[0]]+=(d_CoMx-d_CoM2.iloc[50+2*one_step,0])
            d_motion = pd.concat([d_motion[0:step+1],d_motion2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            Rd_end = pd.concat([Rd_end[0:step+1],Rd_end2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            Ld_end = pd.concat([Ld_end[0:step+1],Ld_end2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            d_CoM = pd.concat([d_CoM[0:step+1],d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            d_CoMv = pd.concat([d_CoMv[0:step+1],d_CoMv2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            RR = np.concatenate((RR[0:step+1, :],data.RR[50+2*one_step:50+4*one_step, :]), axis=0)
            LR = np.concatenate((LR[0:step+1, :],data.LR[50+2*one_step:50+4*one_step, :]), axis=0)
            Rd_end_w=pd.concat([Rd_end_w[0:step+1],Rd_end2[50+2*one_step:50+4*one_step]+d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
            Ld_end_w=pd.concat([Ld_end_w[0:step+1],Ld_end2[50+2*one_step:50+4*one_step]+d_CoM2[50+2*one_step:50+4*one_step]],axis=0,ignore_index=True)
    return d_motion,Rd_end,Ld_end,d_CoM,d_CoMv,RR,LR,Rd_end_w,Ld_end_w