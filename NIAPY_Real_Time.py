import numpy as np
from niapy.task import Task
from niapy.problems import Problem
from niapy.algorithms.basic import ParticleSwarmAlgorithm,GeneticAlgorithm
import pandas as pd
import sys
sys.path.append('C:\\Users\\poetr\\OneDrive\\桌面\\LIPM\\Simulate')
from Simulate.Plot import *
from Simulate.Dataprocess import *
from control.Controller import *
from Simulate.LIPM import *
from Simulate.Simulate_Real_Time import *
import random
mode = 5   # 1==SF 2=FS 3=SFS 4=FSF 5=N
sim_speed=256.0
speed_list=[0.4,0.5,0.6,0.7,0.8]
file_path_n="test1.txt"
file_path_sf="test5.txt"   
file_path_fs="test6.txt"  
speed_combination = []

for i in range(len(speed_list)):
    for j in range(i+1, len(speed_list)):
        if speed_list[i] < speed_list[j]:
            speed_combination.append([speed_list[j],speed_list[i]])

LIPM_data = {}
LIPM_motion = {}
for i in speed_list:
    LIPM_data[str(i)] = [LIPM("data\\F8_"+str(i), i, 8),
                                            LIPM_l("data\\F8_"+str(i)+"_l", i, 8)]
    LIPM_motion[str(i)] = [pd.read_csv("Simulate\\data\\F8_"+str(i)+".csv", header=None, index_col=None),
                                                pd.read_csv("Simulate\\data\\F8_"+str(i)+"_l.csv", header=None, index_col=None)]

parameter_num=14
Eval_data=pd.DataFrame(columns=['Evaluations', 'Fitness'])

Sixteen_Step_data=pd.DataFrame(columns=[0.0]*parameter_num)
class MyProblem(Problem):
    def __init__(self, dimension,data,data1,run=0,best_parameter=[0.0]*parameter_num,best_fitness=100):
        #DSP_hip,DSP_ankle,SSP_hip,SSP_ankle #ratio,DSP_ankle,SP_ankle,SSP_ankle #FTC parameter*6
        if mode==1 or mode==2:
            #訓練FS,SF要將前8參數換成CoP Controller得到的最佳化參數
            lower_bounds=[0.019840150494790557,0.05920724882647929,0.05013555213776386,0.012226660359531362,0.010504190399520823,0.000808535916374517,0.011493550277712872,0.023112587163807055,0.0,0.0,0.0,0.0,0.0,0.0]
            upper_bounds=[0.019840150494790557,0.05920724882647929,0.05013555213776386,0.012226660359531362,0.010504190399520823,0.000808535916374517,0.011493550277712872,0.023112587163807055,0.02,0.02,0.02,0.02,0.02,0.02] 
        elif mode==5 :
            lower_bounds=[0.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0,0,0] #roll(DSP_hip,DSP_ankle,SSP_hip,SSP_ankle )/pitch(ratio,DSP_ankle,SP_ankle,SSP_ankle)
            upper_bounds=[0.1,0.1,0.1,0.1,0.025,0.025,0.025,0.025,0,0,0,0,0,0] 
        super().__init__(dimension=dimension, lower=lower_bounds, upper=upper_bounds)
        self.run=run
        self.data=data
        self.data1=data1
        self.balence_step=0
        self.best_parameter=best_parameter
        self.best_fitness=best_fitness
        if mode==1:
            file_path = file_path_sf
        elif mode==2:
            file_path = file_path_fs
        elif mode==5:
            file_path = file_path_n
        self.file_path=file_path
    def _evaluate(self, x):
        print("-"*100)
        print("parameter:[%0.17f ,%0.17f, %0.17f, %0.17f, %0.17f, %0.17f, %0.17f, %0.17f,\n %0.17f, %0.17f, %0.17f, %0.17f, %0.17f, %0.17f]" \
              %(x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[10],x[11],x[12],x[13]))
        random_selection = random.sample(speed_combination, 5)
        ## mode change ##
        if mode==5:
            total_fitness=0
            total_balence_step=0
            for vel in speed_list:
                #要記得重置Cmd
                Cmd=[0,0]
                speed=[vel,0.4]
                changed_step = [random.randint(3, 5) * 2] #6,8,10 #changed_step 更換速度的step EX:8 表示第8步更換速度
                simulation = Simulate_Real_Time(LIPM_data,LIPM_motion,Cmd,speed,mode,sim_speed)
                R_end_err, L_end_err, CoM_err,CoMv_err,i =simulation.Simulate(x,changed_step)
                fitness,balence_step=evaluation_n(R_end_err,L_end_err,CoM_err,CoMv_err,i,mode,changed_step,speed)
                total_fitness+=fitness
                total_balence_step+=balence_step
            total_fitness/=len(speed_list)

        elif mode==1 or mode==2:
            total_fitness=0
            total_balence_step=0

            for rand in random_selection:
                #要記得重置Cmd
                if mode == 1 :
                    Cmd=[0,0]
                elif mode ==2 :
                    Cmd=[1,0]
                speed = rand
                changed_step = [random.randint(3, 5) * 2] #6,8,10

                simulation = Simulate_Real_Time(LIPM_data,LIPM_motion,Cmd,speed,mode,sim_speed)
                R_end_err, L_end_err, CoM_err,CoMv_err,i =simulation.Simulate(x,changed_step)
                if mode==1:
                    fitness,balence_step=evaluation_sf(R_end_err,L_end_err,CoM_err,CoMv_err,i,mode,changed_step,speed)
                elif mode==2:
                    fitness,balence_step=evaluation_fs(R_end_err,L_end_err,CoM_err,CoMv_err,i,mode,changed_step,speed)
                total_fitness+=fitness
                total_balence_step+=balence_step
            total_fitness/=5

        if self.run%10==0: #清理檔案
            with open(self.file_path, 'a') as file:
                file.truncate(0)
            print("\nfile clear\n")

        self.run+=1

        if total_fitness<self.best_fitness:
            self.best_fitness=total_fitness
            self.best_parameter=x
            df = pd.DataFrame(my_problem.best_parameter).T
            if mode==1:
                df.to_csv('best_parameter_sf_GA.csv', header=None,index=None)
            elif mode==2:
                df.to_csv('best_parameter_fs_GA.csv', header=None,index=None)
            elif mode==5:
                df.to_csv('best_parameter_n_GA.csv', header=None,index=None)
        if total_balence_step>self.balence_step :
            self.balence_step=total_balence_step
        
        if total_balence_step>=1200 and mode==5:
            self.data1.loc[len(self.data1)] = x
        elif total_balence_step>=1500 and (mode==1 or 2):
            self.data1.loc[len(self.data1)] = x

        if mode==1:     
            print("\nSlow2Fast iteration: %6d , fitness: %2.5f , best_fit: %2.5f, best_balence_step: %4.1f\n" % (self.run, total_fitness ,my_problem.best_fitness, self.balence_step))
        elif mode==2:
            print("\nFast2Slow iteration: %6d , fitness: %2.5f , best_fit: %2.5f, best_balence_step: %4.1f\n" % (self.run, total_fitness ,my_problem.best_fitness, self.balence_step))
        elif mode==5:
            print("\nNormal iteration: %6d , fitness: %2.5f , best_fit: %2.5f, best_balence_step: %4.1f\n" % (self.run, total_fitness ,my_problem.best_fitness, self.balence_step))     
        self.data.loc[len(self.data)] = [self.run, self.best_fitness]
        if mode==1:
            Eval_data.to_csv("evaluation_sf_GA.csv", index=False)
            Sixteen_Step_data.to_csv("16_step_sf_GA.csv", index=False)
        elif mode==2:
            Eval_data.to_csv("evaluation_fs_GA.csv", index=False)
            Sixteen_Step_data.to_csv("16_step_fs_GA.csv", index=False)
        elif mode==5:
            Eval_data.to_csv("evaluation_n_GA.csv", index=False)
            Sixteen_Step_data.to_csv("20_step_n_GA.csv", index=False)
        return fitness

my_problem = MyProblem(dimension=parameter_num,data=Eval_data,data1=Sixteen_Step_data)
for i in range(1):
    try:
        task = Task(problem=my_problem, max_iters=110)

        # algorithm = ParticleSwarmAlgorithm(population_size=100)
        algorithm = GeneticAlgorithm(population_size=100)
        
        best_x, best_fit = algorithm.run(task)
        print("\nbest_x: ",best_x)
        print("best_fit: ",best_fit)
        if mode==1:
            Eval_data.to_csv("evaluation_sf_GA.csv", index=False)
            Sixteen_Step_data.to_csv("16_step_sf_GA.csv", index=False)
            plot_eval(Eval_data,"Slow to Fast (CoP Control w Compensator)")
        elif mode==2:
            Eval_data.to_csv("evaluation_fs_GA.csv", index=False)
            Sixteen_Step_data.to_csv("16_step_fs_GA.csv", index=False)
            plot_eval(Eval_data,"Fast to Slow (CoP Control w Compensator)")
        elif mode==5:
            Eval_data.to_csv("evaluation_n_GA.csv", index=False)
            Sixteen_Step_data.to_csv("20_step_n_GA.csv", index=False)
            plot_eval(Eval_data,"Slow (CoP Control)")
    except :
        print("\nbest_x: ",my_problem.best_parameter)
        print("best_fit: ",my_problem.best_fitness)
        if mode==1:
            Eval_data.to_csv("evaluation_sf_GA.csv", index=False)
            Sixteen_Step_data.to_csv("16_step_sf_GA.csv", index=False)
            plot_eval(Eval_data,"Slow to Fast (CoP Control w Compensator)")
        elif mode==2:
            Eval_data.to_csv("evaluation_fs_GA.csv", index=False)
            Sixteen_Step_data.to_csv("16_step_fs_GA.csv", index=False)
            plot_eval(Eval_data,"Fast to Slow (CoP Control w Compensator)")
        elif mode==5:
            Eval_data.to_csv("evaluation_n_GA.csv", index=False)
            Sixteen_Step_data.to_csv("20_step_n_GA.csv", index=False)
            plot_eval(Eval_data,"Slow (CoP Control)")