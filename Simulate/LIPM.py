import sys
sys.path.append('C:\\Users\\poetr\\OneDrive\\桌面\\LIPM')
from src.LIPMMotionGenerator import LIPM_motion_generator
import os
import numpy as np
def deg2rad(angle):
    return angle*np.pi/180
# 將弧度換算角度進行控制運算
def rad2deg(radius):
    return radius/np.pi*180  
class LIPM:
    def  __init__(self, fileName='', one_step_time=1, footStep=8):
        self.fileName = fileName
        self.footStep = footStep
        self.motionGen = True
        self.one_step_time= one_step_time
        self.outputData = []
        self.Rd_end = []
        self.Ld_end = []
        self.d_CoM = []
    # def generate_motion(self):
        """ Define type of motion """
        rightFirst = True
        forward = True
        shift = False
        turn = False
        """ Setting of LIPM parameters """
        # Amplitude of swing
        b1 = -0.200
        b2 = 0.200
        b3 = -0.190
        b4 = 0.190
        b5 = -0.190
        b6 = 0.190
        # Motor 31/35 and Motor 41/45
        # 正： 腰與腳踝彎曲方向一致
        hip1 = 0.6
        hip2 = 0.5

        # Step Height
        stepHeight1 = 0.06
        stepHeight2 = 0.06
        stepHeight3 = 0.06
        stepHeight4 = 0.06
        stepHeight5 = 0.06
        stepHeight6 = 0.06
        # Forward Distance
        stepSize1 = 0.05+0.0
        stepSize2 = 0.05+0.0
        stepSize3 = 0.05+0.0
        stepSize4 = 0.05+0.0
        stepSize5 = 0.0
        stepSize6 = 0.0
        # Lateral Displacement
        shift1 = np.array([0]*self.footStep)
        shift2 = np.array([0]*self.footStep)

        # Motor 30 and Motor 40
        yawAngleR = [[0, 0, 0, 0, 0]]*self.footStep
        yawAngleL = [[0, 0, 0, 0, 0]]*self.footStep 

        # Motor 34 and Motor 44
        initLeanAngleR = 0 # 正: 腳尖向下
        initLeanAngleL = 0 # 正: 腳尖向下

        leanAngleR1 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleR2 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleR3 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleR = leanAngleR1+leanAngleR2*int(self.footStep/2-2)+leanAngleR3
        leanAngleL1 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleL2 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleL3 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleL = leanAngleL1+leanAngleL2*int(self.footStep/2-2)+leanAngleL3
        # Motor 35 and Motor 45
        pedalRollAngleR = [[0, 0, 0, 0, 0]]*(self.footStep)
        pedalRollAngleL = [[0, 0, 0, 0, 0]]*(self.footStep)                     
        """ Data Preprocessing """
        B = [b1, b2]+[b3,b4]*int(self.footStep/2-2)+[b5,b6]
        Hip = [hip1, hip2]
        StepHeight = [[stepHeight1, 0]+[stepHeight3, 0]*int(self.footStep/2-2)+[stepHeight5, 0]]+\
                    [[0, stepHeight2]+[0, stepHeight4]*int(self.footStep/2-2)+[0, stepHeight6]]
        StepSize = [stepSize1, stepSize2]+[stepSize3, stepSize4]*int(self.footStep/2-2)+[stepSize5, stepSize6]
        Shift = [shift1, shift2]
        InitLeanAngle = [initLeanAngleR, initLeanAngleL]
        LeanAngle = [leanAngleR, leanAngleL]
        YawAngle = [yawAngleR, yawAngleL]
        pedalRollAngle = [pedalRollAngleR, pedalRollAngleL]
        """ Parameters of robot """
        legLinkLength = [102, 357.95, 366.42, 29, 111.75]
        footHeight = 978
        zCoM = 674.5
        xCoM = 0
        d2 = 6 / 1000
        """ Generate Original Profile """
        kDSP = 0.5 #雙腳著地時間比例
        period = self.one_step_time #走一步的時間 #改0.75不錯
        samplingTime = 0.01

        self.lipm_motion  = LIPM_motion_generator(rightFirst, forward, shift, turn)
        self.lipm_motion.setRobot(legLinkLength, footHeight, zCoM, xCoM, d2)
        self.lipm_motion.setParameters(B, Hip, StepHeight, StepSize, Shift, InitLeanAngle,
                                LeanAngle, YawAngle, pedalRollAngle)
        self.outputData, self.Rd_end, self.Ld_end, self.d_CoM,self.d_CoMv, self.RR, self.LR = self.lipm_motion.gaitGeneration(period=period,
                                                dt=samplingTime,
                                                footStep=self.footStep,
                                                kDSP=kDSP)
        #將desired值符合模擬環境
        self.d_CoM[1] = -self.d_CoM[1] 
        self.d_CoMv[1] = -self.d_CoMv[1] 
        self.Rd_end[1] = -self.Rd_end[1] - 0.105 #相對base右腳軌跡
        self.Ld_end[1] = -self.Ld_end[1] + 0.105 #相對base左腳軌跡
        self.Rd_end_w = self.Rd_end + self.d_CoM #Rd_end 相對世界座標
        self.Ld_end_w = self.Ld_end + self.d_CoM
        """ Generate Motion Data """
        initR = [0, 0, -deg2rad(15.5), deg2rad(32), -deg2rad(16.5), 0] #hip + 前踢/knee + 彎曲 /ankle + 上抬
        initL = [0, 0, -deg2rad(15.5), deg2rad(32), -deg2rad(16.5), 0]
        initPose = [initR, initL]


        scaleR = [1, 1, 1, 1, 1, 1]
        scaleL = [1, 1, 1, 1, 1, 1]
        scale = [scaleR, scaleL]

        # 重要！馬達方向！！
        dirR = [1, -1, 1, 1, 1, -1]
        dirL = [1, -1, 1, 1, 1, -1]
        dir = [dirR, dirL]

        if self.motionGen == True: # Motion 檔 
            currentFolder = os.path.dirname(os.path.abspath(__file__))
            motionFilePath = currentFolder + "\\" + self.fileName
            self.lipm_motion.writeFile(self.outputData, motionFilePath, initPose, scale, dir)

class LIPM_l:
    def  __init__(self, fileName='', one_step_time=1, footStep=8):
        self.fileName = fileName
        self.footStep = footStep
        self.motionGen = True
        self.one_step_time= one_step_time
        self.outputData = []
        self.Rd_end = []
        self.Ld_end = []
        self.d_CoM = []
        """ Define type of motion """
        rightFirst = True
        forward = True
        shift = False
        turn = False
        """ Setting of LIPM parameters """
        # Amplitude of swing
        b1 = -0.200
        b2 = 0.200
        b3 = -0.190
        b4 = 0.190
        b5 = -0.190
        b6 = 0.190
        # Motor 31/35 and Motor 41/45
        # 正： 腰與腳踝彎曲方向一致
        hip1 = 0.6
        hip2 = 0.5

        # Step Height
        stepHeight1 = 0.06
        stepHeight2 = 0.06
        stepHeight3 = 0.06
        stepHeight4 = 0.06
        stepHeight5 = 0.06
        stepHeight6 = 0.06
        # Forward Distance
        stepSize1 = 0.05
        stepSize2 = 0.05
        stepSize3 = 0.05
        stepSize4 = 0.05
        stepSize5 = 0.05
        stepSize6 = 0.0
        # Lateral Displacement
        shift1 = np.array([0]*self.footStep)
        shift2 = np.array([0]*self.footStep)

        # Motor 30 and Motor 40
        yawAngleR = [[0, 0, 0, 0, 0]]*self.footStep
        yawAngleL = [[0, 0, 0, 0, 0]]*self.footStep 

        # Motor 34 and Motor 44
        initLeanAngleR = 0 # 正: 腳尖向下
        initLeanAngleL = 0 # 正: 腳尖向下

        leanAngleR1 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleR2 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleR3 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleR = leanAngleR1+leanAngleR2*int(self.footStep/2-2)+leanAngleR3
        leanAngleL1 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleL2 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleL3 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
        leanAngleL = leanAngleL1+leanAngleL2*int(self.footStep/2-2)+leanAngleL3
        # Motor 35 and Motor 45
        pedalRollAngleR = [[0, 0, 0, 0, 0]]*(self.footStep)
        pedalRollAngleL = [[0, 0, 0, 0, 0]]*(self.footStep)                     
        """ Data Preprocessing """
        B = [b1, b2]+[b3,b4]*int(self.footStep/2-2)+[b5,b6]
        Hip = [hip1, hip2]
        StepHeight = [[stepHeight1, 0]+[stepHeight3, 0]*int(self.footStep/2-2)+[stepHeight5, 0]]+\
                    [[0, stepHeight2]+[0, stepHeight4]*int(self.footStep/2-2)+[0, stepHeight6]]
        StepSize = [stepSize1, stepSize2]+[stepSize3, stepSize4]*int(self.footStep/2-2)+[stepSize5, stepSize6]
        Shift = [shift1, shift2]
        InitLeanAngle = [initLeanAngleR, initLeanAngleL]
        LeanAngle = [leanAngleR, leanAngleL]
        YawAngle = [yawAngleR, yawAngleL]
        pedalRollAngle = [pedalRollAngleR, pedalRollAngleL]
        """ Parameters of robot """
        legLinkLength = [102, 357.95, 366.42, 29, 111.75]
        footHeight = 978
        zCoM = 674.5
        xCoM = 0
        d2 = 6 / 1000
        """ Generate Original Profile """
        kDSP = 0.5 #雙腳著地時間比例
        period = self.one_step_time #走一步的時間 #改0.75不錯
        samplingTime = 0.01

        self.lipm_motion  = LIPM_motion_generator(rightFirst, forward, shift, turn)
        self.lipm_motion.setRobot(legLinkLength, footHeight, zCoM, xCoM, d2)
        self.lipm_motion.setParameters(B, Hip, StepHeight, StepSize, Shift, InitLeanAngle,
                                LeanAngle, YawAngle, pedalRollAngle)
        self.outputData, self.Rd_end, self.Ld_end, self.d_CoM,self.d_CoMv, self.RR, self.LR = self.lipm_motion.gaitGeneration(period=period,
                                                dt=samplingTime,
                                                footStep=self.footStep,
                                                kDSP=kDSP)
        #將desired值符合模擬環境
        self.d_CoM[1] = -self.d_CoM[1] 
        self.d_CoMv[1] = -self.d_CoMv[1] 
        self.Rd_end[1] = -self.Rd_end[1] - 0.105 #相對base右腳軌跡
        self.Ld_end[1] = -self.Ld_end[1] + 0.105 #相對base左腳軌跡
        self.Rd_end_w = self.Rd_end + self.d_CoM #Rd_end 相對世界座標
        self.Ld_end_w = self.Ld_end + self.d_CoM
        """ Generate Motion Data """
        initR = [0, 0, -deg2rad(15.5), deg2rad(32), -deg2rad(16.5), 0] #hip + 前踢/knee + 彎曲 /ankle + 上抬
        initL = [0, 0, -deg2rad(15.5), deg2rad(32), -deg2rad(16.5), 0]
        initPose = [initR, initL]


        scaleR = [1, 1, 1, 1, 1, 1]
        scaleL = [1, 1, 1, 1, 1, 1]
        scale = [scaleR, scaleL]

        # 重要！馬達方向！！
        dirR = [1, -1, 1, 1, 1, -1]
        dirL = [1, -1, 1, 1, 1, -1]
        dir = [dirR, dirL]

        if self.motionGen == True: # Motion 檔 
            currentFolder = os.path.dirname(os.path.abspath(__file__))
            motionFilePath = currentFolder + "\\" + self.fileName
            self.lipm_motion.writeFile(self.outputData, motionFilePath, initPose, scale, dir)