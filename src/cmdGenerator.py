import csv
import numpy as np
from math import *
import pandas as pd

class cmdGenerator:
    def __init__(self, motorIDList, HT03_MotorID, Dynamixel_MotorID):
        self.motorList = motorIDList
        self.HT03_MotorID = HT03_MotorID
        self.Dynamixel_MotorID = Dynamixel_MotorID
        
    def writeFile(self, outputFileName, footStep, motionData=None, motionFile=None):
        for i in range(int(footStep/2)+1):
            # Read motion file
            self.motionData = motionData
            if motionFile is not None:
                inputfilepath =""
                if i==0 or footStep==2:
                    inputfilepath = motionFile + ".csv"
                else:
                    inputfilepath = motionFile + "_" + str(i) + ".csv"
                with open(inputfilepath, 'r', newline='') as r_f:
                    self.motionData = pd.read_csv(r_f, header=None)
                    self.motionData = self.motionData.values.tolist() 
            
            # Data processing
            cmdList = []
            prePose = np.zeros([len(self.motionData[0])])
            for t_idx, tmpPose in enumerate(self.motionData):
                for motor_idx in range(len(tmpPose)):
                    # Motor Id
                    motorID = self.motorList[motor_idx] 

                    # Motor Type
                    if motorID in self.Dynamixel_MotorID:
                        motorType = 5
                    elif motorID//10 == 3:  # for Right Leg
                        motorType = 3
                    elif motorID//10 == 4:  # for Left Leg
                        motorType = 4
                    
                    # Motor Position
                    motorPos = np.round(tmpPose[motor_idx], 3)

                    # Motor Mode
                    motorMode = 1
                    if t_idx == 0 or t_idx == len(self.motionData)-1:
                        motorMode = 0
                    elif motorPos == self.motionData[t_idx+1][motor_idx]:
                        motorMode = 0

                    # Motor Time
                    motorTime = 0.075
                    if t_idx == 0:
                        motorTime = 2
                    
                    # Motor Velocity
                    # Warning: Ratio
                    motorVel = np.round((motorPos - prePose[motor_idx])/motorTime* 2920 / 175.2, 3)

                    # Motor Torque
                    motorTorque = 0

                    # Motor Kp
                    # (if the motion is jumping, we need to set a smaller kp)
                    motorKp = 100

                    # Motor Kd
                    motorKd = 0.2

                    # Write Command
                    if motorPos != prePose[motor_idx] or t_idx == 0 or t_idx == len(self.motionData)-1:
                        if motorID in self.Dynamixel_MotorID:
                            cmd = [motorType, motorID, motorPos, motorVel]
                        else:
                            cmd = [motorType, motorID, motorMode, motorPos, motorTime, motorTorque, motorKp, motorKd]
                        cmdList.append(cmd)

                # Add time sleep every time step
                waitCmd = [100, motorTime*pow(10, 6)]
                cmdList.append(waitCmd)

                # Update Pose
                prePose = tmpPose

            # Add Final time sleep
            waitCmd = [100, 3*pow(10, 6)]
            cmdList.append(waitCmd)

            
            outputfilepath =""
            if i==0 or footStep==2:
                outputfilepath = outputFileName + ".csv"
            else:
                outputfilepath = outputFileName + "_" + str(i) + ".csv"

            # Write Output File
            with open(outputfilepath, 'w', newline='') as w_f:
                writer = csv.writer(w_f)
                for row in cmdList:
                    writer.writerow(row)

        print('Done Command Generating!')


        









                

                

        
            
                
