import os
os.add_dll_directory('C:/Users/poetr/.mujoco/mujoco200/bin')
os.add_dll_directory('C:/Users/poetr/anaconda3/envs/mujoco/Lib/site-packages/mujoco_py')
import mujoco_py
import glfw
import numpy as np
import pandas as pd
import sys
sys.path.append('C:\\Users\\poetr\\OneDrive\\桌面\\LIPM\\Simulate')
from Plot import *
from Dataprocess import *
from control.Controller import *
from LIPM import *
import threading
import time
import random
class Simulate_Real_Time:
    def __init__(self,LIPM_data,LIPM_motion,Cmd,speed,mode,sim_speed=1.0):
        self.file_path_n="test1.txt"
        self.file_path_sf="test5.txt"   
        self.file_path_fs="test6.txt"   
        self.LIPM_data = LIPM_data
        self.LIPM_motion = LIPM_motion
        self.speed = speed #one step speed [slow,fast]
        self.mode = mode
        self.Cmd = Cmd #Cmd = [上個狀態快慢,此時Cmd]
        self.one_step_t=[-1,-1] #[pre_step_duration, step_duration]
        if self.Cmd[0]==0:
            self.one_step_t[1]=self.speed[0]
        else:
            self.one_step_t[1]=self.speed[1]
        self.ori_step=int(self.one_step_t[1]/0.01) #隨Cmd[0]改變
        self.s_step=int(self.speed[0]/0.01) 
        self.f_step=int(self.speed[1]/0.01) 
        self.simulate = True
        self.flag = True
        self.change = False #change data

        """ 模擬的參數 """
        urdf_path = 'Simulate\linkage_robot.xml'
        self.model = mujoco_py.load_model_from_path(urdf_path)
        self.RR=self.LIPM_data[str(self.one_step_t[1])][0].RR
        self.LR=self.LIPM_data[str(self.one_step_t[1])][0].LR
        self.d_motion, self.Rd_end, self.Ld_end, self.d_CoM,self.d_CoMv=Data_preprocess(self.LIPM_motion[str(self.one_step_t[1])][0],50,self.LIPM_data[str(self.one_step_t[1])][0].Rd_end,self.LIPM_data[str(self.one_step_t[1])][0].Ld_end,self.LIPM_data[str(self.one_step_t[1])][0].d_CoM,self.LIPM_data[str(self.one_step_t[1])][0].d_CoMv)
        self.d_motion=self.d_motion[0:50+self.ori_step*2]
        self.Rd_end_w=self.Rd_end+self.d_CoM
        self.Ld_end_w=self.Ld_end+self.d_CoM

        # 創建Mujoco仿真環境
        self.sim = mujoco_py.MjSim(self.model)
        # 顯示Mujoco仿真視窗
        self.viewer = mujoco_py.MjViewer(self.sim)
        self.window = self.viewer.window
        # 仿真步驟
        # 給機器人初始位置
        self.sim.data.qpos[:],self.acc_err = initial_pos()
        self.r_CoM = []  # 紀錄實際 CoM 的軌跡
        self.r_CoMv = [] # 紀錄實際 CoM 的速度軌跡
        self.r_motion = []  # 紀錄實際馬達的軌跡
        self.c_motion = []  # 紀錄經控制器得到的軌跡
        self.c_motion2 = []  # 紀錄經控制器得到的軌跡
        self.r_sensor_data = []  # 紀錄實際感測器數值
        self.d_sole_traj = []  # 紀錄理想腳底板需轉動的軌跡
        self.R_end = []  # 相對 base，右腳軌跡
        self.RIK_end = []  # 相對 base，順項運動學算的右腳軌跡
        self.L_end = []  # 相對 base，左腳軌跡
        self.R_end_err = []
        self.L_end_err = []
        self.CoM_err = []
        self.CoMv_err = []
        self.CoP = []
        self.euler =[]
        self.kp = [160,100,2500]
        self.kv = [0.6,0.6,10]
        self.ki = [0.1,0.1,0]
        self.viewer._run_speed *= sim_speed

        self.stop = len(self.d_motion.iloc[:,0])-1
        self.temp = len(self.d_motion.iloc[:,0])-1
        self.step = 0
        self.d_CoMx = 0
        self.buffer = 10 #串接動作的緩衝time steps
        self.a=1 #左腳停的指標，預設都是左腳停
    def convert_to_ascii(self,s):
        return [ord(char) for char in s]
    def read_last_line(self,file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()
            if lines:
                return lines[-1].strip()
            else:
                return "No Command"
    def mode2cmd(self,command,Cmd): 
        #Cmd=[speed(Slow/Fast),command] [上個動作快慢/此時的User Command]
        #speed 0 Slow/1 Fast 
        #command 0 N /1 SF /2 FS/ 3 ST
        if Cmd[1]==0:
            pass 
        elif Cmd[1]==1:
            Cmd[0]=1
        elif Cmd[1]==2:
            Cmd[0]=0

        if len(command)== 1 :
            Cmd[1]=0 #No change
        elif command[0]==83 and command[1]==84: #Read User Command ASCII code
            Cmd[1]=3 #ST
        elif command[0]==83 and command[1]==70: 
            Cmd[1]=1 #SF
        elif command[0]==70 and command[1]==83: 
            Cmd[1]=2 #FS
        return Cmd
    def job(self):
        #mode 1==SF 2=FS 3=SFS 4=FSF 5=N
        self.d_CoMx=float(self.d_CoM.iloc[self.step+self.buffer,0]) 
        if self.mode==1:
           file_path = self.file_path_sf
        elif self.mode==2:
           file_path = self.file_path_fs
        elif self.mode==5 or 4:
           file_path = self.file_path_n

        last_line = self.read_last_line(file_path)
        command = self.convert_to_ascii(last_line)
        self.Cmd=self.mode2cmd(command,self.Cmd)

        self.one_step_t[0] = self.one_step_t[1]
        if self.Cmd[1] == 1:
            self.one_step_t[1] = self.speed[1]
        elif self.Cmd[1] == 2:
            self.one_step_t[1] = self.speed[0]
        elif self.Cmd[1] == 0:
            if self.Cmd[0] == 0:
                self.one_step_t[1] = self.speed[0]
            else:
                self.one_step_t[1] = self.speed[1]

        if self.Cmd[1] == 3: #ST
            if self.Cmd[0]== 0: #維持慢速
                self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w=Motion_Concat(self.Cmd,self.LIPM_data[str(self.speed[0])][0],self.LIPM_motion[str(self.speed[0])][0],self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w,self.one_step_t[1],self.step+self.buffer,self.d_CoMx,1) #右腳停
            elif self.Cmd[0]== 1: #維持快速
                self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w=Motion_Concat(self.Cmd,self.LIPM_data[str(self.speed[1])][0],self.LIPM_motion[str(self.speed[1])][0],self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w,self.one_step_t[1],self.step+self.buffer,self.d_CoMx,1)
            self.a=0
            self.simulate=False #準備停止模擬

        if self.Cmd[1]== 0: #N
            if self.Cmd[0]== 0: #維持慢速
                self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w=Motion_Concat(self.Cmd,self.LIPM_data[str(self.speed[0])][0],self.LIPM_motion[str(self.speed[0])][0],self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w,self.one_step_t[1],self.step+self.buffer,self.d_CoMx,1)
            elif self.Cmd[0]== 1: #維持快速
                self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w=Motion_Concat(self.Cmd,self.LIPM_data[str(self.speed[1])][0],self.LIPM_motion[str(self.speed[1])][0],self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w,self.one_step_t[1],self.step+self.buffer,self.d_CoMx,1)
        elif self.Cmd[1]== 1: #SF
            self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w=Motion_Concat(self.Cmd,self.LIPM_data[str(self.speed[1])][0],self.LIPM_motion[str(self.speed[1])][0],self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w,self.one_step_t[1],self.step+self.buffer,self.d_CoMx,1)
        elif self.Cmd[1]== 2: #FS
            self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w=Motion_Concat(self.Cmd,self.LIPM_data[str(self.speed[0])][0],self.LIPM_motion[str(self.speed[0])][0],self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w,self.one_step_t[1],self.step+self.buffer,self.d_CoMx,1)
    def job1(self):
        #mode 1==SF 2=FS 3=SFS 4=FSF 5=N
        self.d_CoMx=float(self.d_CoM.iloc[self.step+self.buffer,0]) 
        if self.mode==1:
           file_path = self.file_path_sf
        elif self.mode==2:
           file_path = self.file_path_fs
        elif self.mode==5 or 4:
           file_path = self.file_path_n

        last_line = self.read_last_line(file_path)
        command = self.convert_to_ascii(last_line)
        self.Cmd=self.mode2cmd(command,self.Cmd)

        if self.Cmd[1] == 3: #ST
            if self.Cmd[0]== 0: #維持慢速
                self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w=Motion_Concat(self.Cmd,self.LIPM_data[str(self.speed[0])][1],self.LIPM_motion[str(self.speed[0])][1],self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w,self.one_step_t[1],self.step+self.buffer,self.d_CoMx,2) #左腳停
            elif self.Cmd[0]== 1: #維持快速
                self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w=Motion_Concat(self.Cmd,self.LIPM_data[str(self.speed[1])][1],self.LIPM_motion[str(self.speed[1])][1],self.d_motion,self.Rd_end,self.Ld_end,self.d_CoM,self.d_CoMv,self.RR,self.LR,self.Rd_end_w,self.Ld_end_w,self.one_step_t[1],self.step+self.buffer,self.d_CoMx,2)
            self.simulate=False #準備停止模擬

    def Simulate(self,parameter, changed_step):
        
        self.weight_roll=parameter[0:4]
        self.weight_pitch=parameter[4:8]
        self.weight=parameter[8:]
        self.over=False
        try:
            while(self.flag):
                if self.simulate==True:
                    if self.step == self.stop-self.buffer : #換資料/左腳停
                        t = threading.Thread(target = self.job())
                        t.start()
                    if self.step == self.temp+int((self.stop-self.temp)/2-self.buffer) : #右腳停
                        if self.a: #左腳停的指標關掉，即右腳停
                            t1 = threading.Thread(target = self.job1())
                            t1.start()

                sensor_data = []   #紀錄壓感資料
                for j in range(8):
                    sensor_data.append(self.sim.data.sensordata[j])
                sensor_pos = [self.sim.data.site_xpos[1],self.sim.data.site_xpos[2],self.sim.data.site_xpos[3],self.sim.data.site_xpos[4], #感測器位置 (左腳前/左腳後/右腳前/右腳後/IMU)
                            self.sim.data.site_xpos[6],self.sim.data.site_xpos[7],self.sim.data.site_xpos[8],self.sim.data.site_xpos[9],self.sim.data.body_xpos[1]]    
                sole_joint_pos = [self.sim.data.geom_xpos[25],self.sim.data.geom_xpos[27],self.sim.data.geom_xpos[49],self.sim.data.geom_xpos[51]] #彈簧位置 (左腳前/左腳後/右腳前/右腳後)
                pos = [self.sim.data.qpos[7], self.sim.data.qpos[8], self.sim.data.qpos[9], self.sim.data.qpos[10], self.sim.data.qpos[13], self.sim.data.qpos[14],
                    self.sim.data.qpos[15], self.sim.data.qpos[18], self.sim.data.qpos[19], self.sim.data.qpos[20], self.sim.data.qpos[23], self.sim.data.qpos[24], self.sim.data.qpos[25]]
                vel = [self.sim.data.qvel[6], self.sim.data.qvel[7], self.sim.data.qvel[8], self.sim.data.qvel[9], self.sim.data.qvel[12], self.sim.data.qvel[13],
                    self.sim.data.qvel[14], self.sim.data.qvel[17], self.sim.data.qvel[18], self.sim.data.qvel[19], self.sim.data.qvel[22], self.sim.data.qvel[23], self.sim.data.qvel[24]]

                #確認腳底板是否觸地(觸地即picth方向需平行)/確認腳是否懸空
                parallel,rise=Sole_Status(sensor_data)
                #腳底板控制 
                d_angle,self.model=Sole_control(self.model,sole_joint_pos,sensor_pos,parallel)
                #紀錄狀態
                self.R_end.append(self.sim.data.site_xpos[5]-sensor_pos[8])
                self.L_end.append(self.sim.data.site_xpos[0]-sensor_pos[8])
                self.r_CoM.append(sensor_pos[8]-[0,0,0])
                self.r_CoMv.append(self.sim.data.cvel[1][3:]-[0,0,0])
                self.r_motion.append(pos)
                self.d_sole_traj.append(d_angle)
                self.r_sensor_data.append(sensor_data)
                self.euler.append([rad2deg(i) for i in list(tfs.euler.quat2euler(self.sim.data.qpos[3:7],"sxyz"))])
                if self.step<2:
                    xy_cop=CoP_compute(self.r_CoM[self.step],[[0.0,0.0],[0.0,0.0]],sensor_pos[0:8],sensor_data,rise)
                else:
                    xy_cop=CoP_compute(self.r_CoM[self.step],[self.CoP[self.step-1],self.CoP[self.step-2]],sensor_pos[0:8],sensor_data,rise)
                self.CoP.append(xy_cop)

                #開始控制
                input=self.d_motion.loc[self.step].copy()
                if self.step>49+0*self.ori_step:    
                    ctrl=self.d_motion.loc[self.step].tolist() #給予馬達目標角度 
                    ctrl[2],ctrl[6],ctrl[8],ctrl[12]=y_CoP_control(input,rise,self.CoP[self.step][1],self.Rd_end_w.loc[self.step][1],self.Ld_end_w.loc[self.step][1],self.weight_roll)
                    ctrl[5],ctrl[11],ctrl[3],ctrl[4],ctrl[9],ctrl[10]=x_CoP_control(input,rise,self.CoP[self.step][0],self.Rd_end_w.loc[self.step][0],self.Ld_end_w.loc[self.step][0],self.weight_pitch)
                    flag=False
                    if (self.step<=len(self.RR)+50) and (self.step>49+2*self.ori_step):
                        input=self.d_motion.loc[self.step-2:self.step].copy()
                        a=[ctrl[3],ctrl[4],ctrl[9],ctrl[10]]
                        if self.mode==1:
                            ctrl[3],ctrl[4],ctrl[9],ctrl[10],flag=Pattern_Change_SF(input,rise,self.Rd_end.loc[self.step].copy(),self.Ld_end.loc[self.step].copy(),self.RR[self.step-51],self.LR[self.step-51],self.euler[self.step],self.Cmd,[ctrl[3],ctrl[4],ctrl[9],ctrl[10]],self.weight)
                        elif self.mode==2:
                            ctrl[3],ctrl[4],ctrl[9],ctrl[10],flag=Pattern_Change_FS(input,rise,self.Rd_end.loc[self.step].copy(),self.Ld_end.loc[self.step].copy(),self.RR[self.step-51],self.LR[self.step-51],self.euler[self.step],self.Cmd,a,self.weight)
                        elif self.mode==3 or self.mode==4:
                            ctrl[3],ctrl[4],ctrl[9],ctrl[10],flag=Pattern_Change_Ctrl(input,rise,self.Rd_end.loc[self.step].copy(),self.Ld_end.loc[self.step].copy(),self.RR[self.step-51],self.LR[self.step-51],self.euler[self.step],self.Cmd,[ctrl[3],ctrl[4],ctrl[9],ctrl[10]],self.weight)
                    if flag:
                        self.c_motion.append(ctrl-self.d_motion.loc[self.step].copy())
                        self.c_motion2.append([0]*13)
                    else:
                        self.c_motion.append([0]*13)
                        self.c_motion2.append(ctrl-self.d_motion.loc[self.step].copy())
                else:
                    ctrl=self.d_motion.loc[self.step].tolist() #給予馬達目標角度
                    self.c_motion.append(ctrl-self.d_motion.loc[self.step].copy())
                    self.c_motion2.append(ctrl-self.d_motion.loc[self.step].copy())

                self.sim.data.ctrl[:],self.acc_err= PID_control(self.kp, self.kv, self.ki ,pos, vel, ctrl, self.acc_err,rise)

                self.R_end_err.append(self.Rd_end.loc[self.step]-self.R_end[self.step])
                self.L_end_err.append(self.Ld_end.loc[self.step]-self.L_end[self.step])
                self.CoM_err.append(self.d_CoM.loc[self.step]-self.r_CoM[self.step])
                self.CoMv_err.append(self.d_CoMv.loc[self.step]-self.r_CoMv[self.step])

                self.sim.step()
                self.viewer.render()

                #跌倒狀況
                if sensor_pos[8][2]<0.9 :
                    self.Rd_end_w= self.Rd_end_w.iloc[:,0:self.step]
                    self.Ld_end_w= self.Ld_end_w.iloc[:,0:self.step]
                    glfw.destroy_window(self.viewer.window)
                    raise 

                if len(self.d_motion.iloc[:,0])-1!=self.stop:
                    self.temp = self.stop
                    self.stop = len(self.d_motion.iloc[:,0])-1 #更新停止條件
                    self.change=True
   
                if self.change:
                    t.join
                    self.change=False

                if self.step==self.stop:
                    self.flag=False

                if self.mode==1:#SF
                    if self.step == 10:
                        with open(self.file_path_sf, 'a') as file:
                            file.write('N\n')
                    elif self.step == 50+int(changed_step[0]-1)*self.s_step: #changed_step 更換速度的step EX:8 表示第8步更換速度
                        with open(self.file_path_sf, 'a') as file:
                            file.write('SF\n')
                    elif self.step == 50+int(changed_step[0])*self.s_step+int(16-changed_step[0]-1.5)*self.f_step: #訓練用，走16步
                        print("In mode 1")
                        with open(self.file_path_sf, 'a') as file:
                            file.write('ST\n')
                    # elif (self.sim.data.geom_xpos[8]-self.sim.data.geom_xpos[1])[0]>-0.3: #測試用，偵測距離自動停止
                    #     with open(self.file_path_sf, 'a') as file:
                    #         file.write('ST\n')

                elif self.mode==2: #FS
                    if self.step == 10:
                        with open(self.file_path_fs, 'a') as file:
                            file.write('N\n')
                    elif self.step == 50+int(changed_step[0]-1)*self.f_step:
                        with open(self.file_path_fs, 'a') as file:
                            file.write('FS\n')
                    elif self.step == 50+int(changed_step[0])*self.f_step+int(16-changed_step[0]-1.5)*self.s_step: #訓練用，走16步
                        print("In mode 2")
                        with open(self.file_path_fs, 'a') as file:
                            file.write('ST\n')
                    # elif (self.sim.data.geom_xpos[8]-self.sim.data.geom_xpos[1])[0]>-0.3: #測試用，偵測距離自動停止
                    #     with open(self.file_path_fs, 'a') as file:
                    #         file.write('ST\n')

                elif self.mode== 5 : #N
                    if self.step == 10:
                        with open(self.file_path_n, 'a') as file:
                            file.write('N\n')
                    elif self.step == 18*self.s_step+50: #訓練用，走20步
                        with open(self.file_path_n, 'a') as file:
                            file.write('ST\n')
                    # elif (self.sim.data.geom_xpos[8]-self.sim.data.geom_xpos[1])[0]>-0.3: #測試用，偵測距離自動停止
                    #     with open(self.file_path_n, 'a') as file:
                    #         file.write('ST\n')

                elif self.mode== 4 : #FSF
                    if self.step == 10:
                        with open(self.file_path_n, 'a') as file:
                            file.write('N\n')
                    elif self.step == 50+int(changed_step[0]-1)*self.f_step:
                        with open(self.file_path_n, 'a') as file:
                            file.write('FS\n')
                    elif self.step == 50+int(changed_step[0])*self.f_step+int(changed_step[1]-changed_step[0]-1)*self.s_step: 
                        with open(self.file_path_n, 'a') as file:
                            file.write('SF\n')
                    if (self.sim.data.geom_xpos[8]-self.sim.data.geom_xpos[1])[0]>-0.3: #偵測距離自動停止
                        print("stop",(self.sim.data.geom_xpos[8]-self.sim.data.geom_xpos[1])[0])
                        with open(self.file_path_n, 'a') as file:
                            file.write('ST\n')
                self.step+=1
            glfw.destroy_window(self.viewer.window)
            ################ PLOT (訓練時不要開)################ 
            # plot_xy_ZMP(self.R_end,self.L_end,self.r_CoM,self.r_CoMv,self.CoP)
            # plot_ctrl_mount(self.c_motion2,self.c_motion) #CoP,FTC variations
            # plot_euler(self.euler) 
            ################ PLOT (訓練時不要開)################

            return self.R_end_err, self.L_end_err, self.CoM_err,self.CoMv_err,self.step
        except : #訓練時沒有完成任務會進到這，計算各種error以及跌倒時的time steps，進行fitness計算

            ################ PLOT (訓練時不要開)################        
            if self.mode==1: #SF
                if self.step<(50+changed_step[0]*self.s_step): #慢到快時
                    step=self.step-(self.step-49)%self.s_step
                    self.R_end_err=self.R_end_err[0:step]
                    self.L_end_err=self.L_end_err[0:step]
                    self.CoM_err=self.CoM_err[0:step]
                else:
                    step=self.step-(self.step-(49+changed_step[0]*self.s_step))%self.f_step
                    self.R_end_err=self.R_end_err[0:step]
                    self.L_end_err=self.L_end_err[0:step]
                    self.CoM_err=self.CoM_err[0:step]
            elif self.mode==2: #FS
                if self.step<(50+changed_step[0]*self.f_step): #快到慢時
                    step=self.step-(self.step-49)%self.f_step
                    self.R_end_err=self.R_end_err[0:step]
                    self.L_end_err=self.L_end_err[0:step]
                    self.CoM_err=self.CoM_err[0:step]
                else:
                    step=self.step-(self.step-(49+changed_step[0]*self.f_step))%self.s_step
                    self.R_end_err=self.R_end_err[0:step]
                    self.L_end_err=self.L_end_err[0:step]
                    self.CoM_err=self.CoM_err[0:step]
            elif self.mode==5: #N
                step=self.step-(self.step-49)%self.s_step
                self.R_end_err=self.R_end_err[0:step]
                self.L_end_err=self.L_end_err[0:step]
                self.CoM_err=self.CoM_err[0:step]


            ################ PLOT (訓練時不要開)################ 
            # plot_xy_ZMP(self.R_end,self.L_end,self.r_CoM,self.r_CoMv,self.CoP)
            # plot_ctrl_mount(self.c_motion2,self.c_motion) #CoP,FTC variations
            # plot_euler(self.euler) 
            ################ PLOT (訓練時不要開)################

            return self.R_end_err, self.L_end_err, self.CoM_err,self.CoMv_err,self.step

if __name__ == "__main__":
    
    mode = 5    # 1==SF 2=FS 3=SFS 4=FSF 5=N
    if mode == 1 or mode ==3 or mode ==5:
        Cmd=[0,0]
    elif mode ==2 or mode ==4 :
        Cmd=[1,0]

    speed_list=[0.4,0.5,0.6,0.7,0.8]
    speed_combination = []

    for i in range(len(speed_list)):
        for j in range(i+1, len(speed_list)):
            if speed_list[i] < speed_list[j]:
                speed_combination.append([speed_list[j],speed_list[i]])

    random_selection = random.sample(speed_combination, 5)

    sim_speed=2
    changed_step=[0]  #changed_step 更換速度的step (這邊初始化用，不用改)
    LIPM_data = {}
    LIPM_motion = {}
    for i in speed_list:

        LIPM_data[str(i)] = [LIPM("data\\F8_"+str(i), i, 8),
                                                LIPM_l("data\\F8_"+str(i)+"_l", i, 8)]
        LIPM_motion[str(i)] = [pd.read_csv("Simulate\\data\\F8_"+str(i)+".csv", header=None, index_col=None),
                                                pd.read_csv("Simulate\\data\\F8_"+str(i)+"_l.csv", header=None, index_col=None)]


    parameter=[0.019840150494790557,0.05920724882647929,0.05013555213776386,0.012226660359531362,0.010504190399520823,0.000808535916374517,0.011493550277712872,0.023112587163807055,0.0012871125148175456,0.012188732209252148,0.0045621288812360454,0.007288547116483844,0.01232961472325166,0.017530323716643294
               ]
    
    total_fitness=0
    if mode==5:
        for vel in speed_list:
            speed=[vel,0.5]
            simulation = Simulate_Real_Time(LIPM_data,LIPM_motion,Cmd,speed,mode,sim_speed)
            R_end_err, L_end_err, CoM_err,CoMv_err,i =simulation.Simulate(parameter,changed_step)
            fitness,balence_step=evaluation_n(R_end_err,L_end_err,CoM_err,CoMv_err,i,mode,changed_step,speed)
            total_fitness+=fitness
        total_fitness/=len(speed_list)
    elif mode==1 or mode==2:
        random_selection = random.sample(speed_combination, 5)
        for rand in random_selection:
            speed = rand #隨機給speed
            #要記得重置Cmd
            if mode == 1 :
                Cmd=[0,0]
            elif mode ==2 :
                Cmd=[1,0]
            changed_step=[random.randint(2, 5) * 2] #隨機給changed_step

            speed = [0.7,0.4] #指定speed
            changed_step=[6] #指定changed_step

            print(mode,speed,changed_step)
            simulation = Simulate_Real_Time(LIPM_data,LIPM_motion,Cmd,speed,mode,sim_speed)
            R_end_err, L_end_err, CoM_err,CoMv_err,i =simulation.Simulate(parameter,changed_step)
            if mode==1:
                fitness,balence_step=evaluation_sf(R_end_err,L_end_err,CoM_err,CoMv_err,i,mode,changed_step,speed)
            elif mode==2:
                fitness,balence_step=evaluation_fs(R_end_err,L_end_err,CoM_err,CoMv_err,i,mode,changed_step,speed)
            break
    elif mode==3 or mode==4:  #3=SFS 4=FSF  
        #要記得重置Cmd
        if mode == 3 :
            Cmd=[0,0]
        elif mode ==4 :
            Cmd=[1,0]
        speed=[0.7,0.5]     
        # changed_step=[random.randint(2, 3) * 2,random.randint(4, 7) * 2] #隨機給changed_step
        changed_step=[6,12] #指定changed_step
        print(mode,speed,changed_step)
        simulation = Simulate_Real_Time(LIPM_data,LIPM_motion,Cmd,speed,mode,sim_speed)
        R_end_err, L_end_err, CoM_err,CoMv_err,i =simulation.Simulate(parameter,changed_step)
  