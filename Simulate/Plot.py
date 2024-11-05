import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
joint_name = [r'$L_{1}$', r'$L_{2}$', r'$L_{3}$', r'$L_{4}$', r'$L_{5}$', r'$L_{6}$', r'$R_{1}$', r'$R_{2}$', r'$R_{3}$', r'$R_{4}$', r'$R_{5}$', r'$R_{6}$']

def plot_d_joint(motion):#印每顆馬達的角度圖
    for i in range(1,13):
        plt.subplot(2, 6, i)
        motion[i].plot()
        plt.xlabel('step')
        plt.ylabel('radius')
        plt.title(joint_name[i-1])
    plt.show()
######joint state######
def plot_joint(d_motion,r_motion):#印每顆馬達期望/實際的角度圖
    r_motion = np.array(r_motion)
    print(r_motion)
    for i in range(1,13):
        plt.subplot(2, 6, i)
        x = np.arange(0, len(r_motion[:,i]), 1)
        y = r_motion[:,i]
        d_motion[i].plot()
        plt.plot(x, y)
        plt.xlabel('step')
        plt.ylabel('radius')
        plt.title(joint_name[i-1])
        plt.legend(["d_motion","r_motion"])
    plt.show()

def plot_ctrl_joint(d_motion,c_motion,r_motion):#印每顆馬達期望/經控制器/實際的角度圖
    r_motion = np.array(r_motion)
    d_motion = np.array(d_motion)
    c_motion = np.array(c_motion)
    for i in range(1,13):
        plt.subplot(2, 6, i)
        x = np.arange(0, len(r_motion[:,i]), 1)
        x1 = np.arange(0, len(d_motion[:,i]), 1)
        y = c_motion[:,i]
        y1= r_motion[:,i]
        y2= d_motion[:len(r_motion[:,i]),i]
        plt.plot(x, y2)
        plt.plot(x, y)
        plt.plot(x, y1)
        plt.xlabel('step')
        plt.ylabel('radius')
        plt.title(joint_name[i-1])
        plt.legend(["d_motion","c_motion","c_motion_HI"])
    plt.show()
######CoM######
def plot_CoM(d_CoM,r_CoM):
    r_CoM = np.array(r_CoM)
    for i in range(0,3):
        label=['CoMx','CoMy','CoMz']
        plt.subplot(1, 3, i+1)
        x = np.arange(0, len(r_CoM[:,i]) , 1)
        # y = d_CoM[i]
        y1= r_CoM[:,i]
        # plt.plot(x, y)
        d_CoM[i].plot()
        plt.plot(x, y1)
        plt.xlabel('step')
        plt.ylabel('m')
        plt.title(label[i])
        plt.legend(["d_CoM","r_CoM"])
    plt.show()

######end-effector state######
def plot_end_effector(Rd_end,R_end,Ld_end,L_end):
    R_end = np.array(R_end)
    L_end = np.array(L_end)
    for i in range(0,6):
        label=['R_foot_x','R_foot_y','R_foot_z','L_foot_x','L_foot_y','L_foot_z']
        legend=[["Rd_end","R_end"],["Ld_end","L_end"]]
        plt.subplot(2, 3, i+1)
        x = np.arange(0, len(R_end[:,i%3]) , 1)
        if i<3:
            # y = Rd_end[i]
            Rd_end[i].plot()
            y1= R_end[:,i]
        else:
            # y = Ld_end[i%3]
            Ld_end[i%3].plot()
            y1= L_end[:,i%3]
        # plt.plot(x, y)
        plt.plot(x, y1)
        plt.xlabel('step')
        plt.ylabel('end-effector')
        plt.title(label[i])
        if i<3:
            plt.legend(legend[0])
        else:
            plt.legend(legend[1])
    plt.show()

######sole state######
def plot_sole(d_sole_traj):
    d_sole_traj = np.array(d_sole_traj)
    for i in range(0,4):
        sole_name = ['L_f_front', 'L_f_back','R_f_front', 'R_f_back']
        plt.subplot(2, 2, i+1)
        x = np.arange(0, len(d_sole_traj[:,i]), 1)
        y = d_sole_traj[:,i]*180/np.pi
        plt.plot(x, y)
        plt.xlabel('step')
        plt.ylabel('degree')
        plt.title(sole_name[i])
    plt.show()

######sensor######
def plot_sensor(r_sensor_data):
    r_sensor_data = np.array(r_sensor_data)
    for i in range(0,8): 
        sensor_name = ['LFR', 'LFL','LBR', 'LBL','RFR', 'RFL','RBR', 'RBL']
        plt.subplot(2, 4, i+1)
        x = np.arange(0, len(r_sensor_data[:,i]), 1)
        y =  r_sensor_data[:,i]*180/np.pi
        plt.plot(x, y)
        plt.xlabel('step')
        plt.ylabel('degree')
        plt.title(sensor_name[i])
    plt.show()


######evaluation######
def plot_eval(data,legend,title):
    plt.rcParams["font.family"] = "Times New Roman"
    for i in range(len(data)):
        plt.plot(data[i]['Evaluations'], data[i]['Fitness'], marker='', linestyle='-')
        plt.title(title,fontsize=26)
        plt.legend(legend,fontsize=20,loc=1)
        plt.xlabel("evaluations",fontsize=26)
        plt.ylabel("fitness",fontsize=26)
        plt.grid(linestyle='--')
    plt.show()


######ZMP_plot######
def plot_xy_ZMP(Rd_end,Ld_end,r_CoM,r_CoMv,CoP):
    plt.rcParams['font.serif'] = ['Times New Roman']
    Rd_end = np.array(Rd_end)
    Ld_end = np.array(Ld_end)
    r_CoMv = np.array(r_CoMv)
    r_CoM = np.array(r_CoM)
    CoP = np.array(CoP)
    Rd_end+=r_CoM
    Ld_end+=r_CoM
    legend=[[r'$foot_{R}^{f}$',r'$foot_{R}^{b}$',r'$CoP_{x}$',r'$foot_{L}^{f}$',r'$foot_{L}^{b}$'],[r'$foot_{R}^{o}$',r'$foot_{R}^{i}$',r'$CoP_{y}$',r'$foot_{L}^{o}$',r'$foot_{L}^{i}$']]
    label = [r'$CoP_{x}$', r'$CoP_{y}$']
    a=["yellow","orange"]
    b=40 #一步的time steps ex.一步0.8s 0.8/0.02=40
    c=350 #平衡time steps+initial steps
    d=70 #改速度後，一步的time steps ex.一步1.4s 1.4/0.02=70
    ch=6 #第一次改速度的step
    ch2=12 #第二次改速度的step
    y_sum_error=0 
    y_error=0
    n_dx_vel=0 #Vel_N
    single_dx_vel=0 #Vel_single
    dual_dx_vel=0 #Vel_dual

    for i in range(0,2): 
        plt.subplot(1, 2, i+1)
        if i==0:
            x = np.arange(0, len(CoP[:,i]) , 1)
            y1= Rd_end[:,i]+0.122
            y2= Rd_end[:,i]-0.078
            y3= CoP[:,i]
            y4= Ld_end[:,i]+0.122
            y5= Ld_end[:,i]-0.078       
            plt.plot(x, y1,color=(0, 0, 1))
            plt.plot(x, y2,color=(0, 0, 1),linestyle='--')
            plt.plot(x, y3,color=(0, 0.7, 0), linewidth=3)
            plt.plot(x, y4,color=(1, 0, 0))
            plt.plot(x, y5,color=(1, 0, 0),linestyle='--')            
            plt.title(label[i],fontsize=26)
            plt.legend(legend[i],fontsize=20,loc=2)
            plt.xlabel('time step (0.02s)',fontsize=26)
            plt.ylabel('stride (m)',fontsize=26)
            plt.grid(linestyle='--')
            ############### N #################
            # for j in range(int((len(CoP[:,i])-c)/b)): 
            #     if (j+1)*b<(len(CoP[:,i])-c):
            #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
            #         n_dx_vel=sum(r_CoMv[50:50+(j+1)*b,i])/(b*(j+1))
            #     else:
            #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color="red",alpha=0.2)
            #         # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
                    
            ############### SF, FS #################
            # for j in range(int((len(CoP[:,i])-c-ch*b)/d)+ch):
            #     if j<ch:
            #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
            #         n_dx_vel=sum(r_CoMv[50:50+(j+1)*b,i])/(b*(j+1))
            #     elif ch*b+(j-ch+1)*d<(len(CoP[:,i])-c):
            #         plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color=a[j%2],alpha=0.2)
            #         single_dx_vel=sum(r_CoMv[50+ch*b:50+ch*b+(j-ch+1)*d,i])/(d*(j+1-ch))
            #     else:
            #         plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color="red",alpha=0.2)
            #         # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
            #         # single_dx_vel=sum(r_CoMv[50+ch*b:50+ch*b+(j-ch+1)*d,i])/(d*(j+1-ch))
            
            ############### SFS, FSF #################
            for j in range(int((len(CoP[:,i])-c-ch*b-(ch2-ch)*d)/b)+ch2):
                if j<ch:
                    plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
                    n_dx_vel=sum(r_CoMv[0:0+(j+1)*b,i])/(b*(j+1))
                elif j<ch2:
                    plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color=a[j%2],alpha=0.2)
                    single_dx_vel=sum(r_CoMv[50+ch*b:50+ch*b+(j-ch+1)*d,i])/(d*(j+1-ch))
                elif ch*b+(ch2-ch)*d+(j-ch2+1)*b<(len(CoP[:,i])-c):
                    plt.axvspan(xmin=50+ch*b+(ch2-ch)*d+(j-ch2)*b,xmax=50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,color=a[j%2],alpha=0.2)
                else:
                    plt.axvspan(xmin=50+ch*b+(ch2-ch)*d+(j-ch2)*b,xmax=50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,color="red",alpha=0.2)
                    # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
                    dual_dx_vel=sum(r_CoMv[50+ch*b+(ch2-ch)*d:50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,i])/(b*(j+1-ch2))

            plt.axvspan(xmin=50+len(CoP[:,i])-c,xmax=len(CoP[:,i]),color="green",alpha=0.2)
        else:
            x = np.arange(0, len(CoP[:,i]) , 1)
            y1= Rd_end[:,i]-0.06
            y2= Rd_end[:,i]+0.06
            y3= CoP[:,i]
            y4= Ld_end[:,i]+0.06
            y5= Ld_end[:,i]-0.06
            plt.plot(x, y1,color=(0, 0, 1))
            plt.plot(x, y2,color=(0, 0, 1),linestyle='--')
            plt.plot(x, y3,color=(0, 0.7, 0), linewidth=3)
            plt.plot(x, y4,color=(1, 0, 0))
            plt.plot(x, y5,color=(1, 0, 0),linestyle='--')
            plt.title(label[i],fontsize=26)
            plt.legend(legend[i],fontsize=20,loc=1)
            plt.xlabel('time step (0.02s)',fontsize=26)
            plt.ylabel('swing (m)',fontsize=26)
            plt.grid(linestyle='--')
            ############### N #################
            # for j in range(int((len(CoP[:,i])-c)/b)):
            #     if (j+1)*b<(len(CoP[:,i])-c):
            #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
            #     else:
            #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color="red",alpha=0.2)
            #         # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)

            ############### SF, FS #################
            # for j in range(int((len(CoP[:,i])-c-ch*b)/d)+ch):
            #     if j<ch:
            #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
            #     elif ch*b+(j-ch+1)*d<(len(CoP[:,i])-c):
            #         plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color=a[j%2],alpha=0.2)
            #     else:
            #         plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color="red",alpha=0.2)
            #         # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)

            ############### SFS, FSF #################
            for j in range(int((len(CoP[:,i])-c-ch*b-(ch2-ch)*d)/b)+ch2):
                if j<ch:
                    plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
                elif j<ch2:
                    plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color=a[j%2],alpha=0.2)
                elif ch*b+(ch2-ch)*d+(j-ch2+1)*b<(len(CoP[:,i])-c):
                    plt.axvspan(xmin=50+ch*b+(ch2-ch)*d+(j-ch2)*b,xmax=50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,color=a[j%2],alpha=0.2)
                else:
                    plt.axvspan(xmin=50+ch*b+(ch2-ch)*d+(j-ch2)*b,xmax=50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,color="red",alpha=0.2)
                    # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)

            plt.axvspan(xmin=50+len(CoP[:,i])-c,xmax=len(CoP[:,i]),color="green",alpha=0.2)
            y_sum_error=sum(CoP[:,i])/len(CoP[:,i])
            y_error=CoP[len(CoP[:,i])-1,i]
    plt.show()  
    print("y_ave_error",y_sum_error)
    print("y_end_error",y_error)
    print("n_dx_vel",n_dx_vel)
    print("single_dx_vel",single_dx_vel)
    print("dual_dx_vel",dual_dx_vel)

def plot_euler(euler):
    plt.rcParams["font.family"] = "Times New Roman"
    euler = np.array(euler).T

    a=["yellow","orange"]
    b=40 #一步的time steps ex.一步0.8s 0.8/0.02=40
    c=350 #平衡time steps+initial steps
    d=70 #改速度後，一步的time steps ex.一步1.4s 1.4/0.02=70
    ch=6 #第一次改速度的step
    ch2=12 #第二次改速度的step
    x = np.arange(0, len(euler[1,:]) , 1)
    y= euler[1,:]
    plt.plot(x, y)
    plt.xlabel('time step (0.02s)',fontsize=26)
    plt.ylabel('tilt angle (degree)',fontsize=26)
    plt.grid(linestyle='--')
    plt.fill_between(x, y, 3, where=(y > 3), color='red', alpha=0.5)
    plt.fill_between(x, y, -3, where=(y < -3), color='red', alpha=0.5)
    ############### N #################
    # for j in range(int((len(euler[1,:])-c)/b)):
    #     if (j+1)*b<(len(euler[1,:])-c):
    #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
    #     else:
    #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color="red",alpha=0.2)
    #         # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)

    ############### SF, FS #################
    # for j in range(int((len(euler[1,:])-c-ch*b)/d)+ch):
    #     if j<ch:
    #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
    #     elif ch*b+(j-ch+1)*d<(len(euler[1,:])-c):
    #         plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color=a[j%2],alpha=0.2)
    #     else:
    #         plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color="red",alpha=0.2)
    #         # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)

    ############### SFS, FSF #################
    for j in range(int((len(euler[1,:])-c-ch*b-(ch2-ch)*d)/b)+ch2):
        if j<ch:
            plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
        elif j<ch2:
            plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color=a[j%2],alpha=0.2)
        elif ch*b+(ch2-ch)*d+(j-ch2+1)*b<(len(euler[1,:])-c):
            plt.axvspan(xmin=50+ch*b+(ch2-ch)*d+(j-ch2)*b,xmax=50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,color=a[j%2],alpha=0.2)
        else:
            plt.axvspan(xmin=50+ch*b+(ch2-ch)*d+(j-ch2)*b,xmax=50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,color="red",alpha=0.2)
            # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)

    plt.axvspan(xmin=50+len(euler[1,:])-c,xmax=len(euler[1,:]),color="green",alpha=0.2)
    plt.title('Euler Angle',fontsize=26)
    plt.legend(['pitch angle'],fontsize=20,loc=1)
    plt.show()

def plot_ctrl_mount(c_cop,c_ftc):
    plt.rcParams["font.family"] = "Times New Roman"
    c_cop = np.array(c_cop)
    c_ftc = np.array(c_ftc)
    n_cop=0
    n_ftc=0
    fs_cop=0
    fs_ftc=0
    sf_cop=0
    sf_ftc=0
    st_cop=0
    st_ftc=0
    b_cop=0
    b_ftc=0
    a=["yellow","orange"]
    b=40 #一步的time steps ex.一步0.8s 0.8/0.02=40
    c=350 #平衡time steps+initial steps
    d=70 #改速度後，一步的time steps ex.一步1.4s 1.4/0.02=70
    ch=6 #第一次改速度的step
    ch2=12 #第二次改速度的step
    for i in range(1,13):
        plt.subplot(2, 6, i)
        x = np.arange(0, len(c_ftc[:,i]), 1)
        if i==2 or i==6 or i==8 or i==12:
            y = c_cop[:,i]
            y1= c_ftc[:,i]-c_ftc[:,i]
        else:
            y = c_cop[:,i]
            y1= c_ftc[:,i]
        plt.plot(x, y)
        plt.plot(x, y1)
        plt.xlabel('time step (0.02s)',fontsize=16)
        plt.ylabel('variation (radian)',fontsize=16)
        plt.grid(linestyle='--')
        plt.title(joint_name[i-1],fontsize=18,fontweight='bold', fontstyle='italic')
        plt.legend(["CoP","FTC"],fontsize=12,loc=1)
        ############### N #################
        # for j in range(int((len(c_ftc[:,i])-c)/b)):
        #     if (j+1)*b<(len(c_ftc[:,i])-c):
        #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
        #         cop_count = [item for item in c_cop[50:50+(j+1)*b,4] if abs(item) > 0]
        #         ftc_count = [item for item in c_ftc[50:50+(j+1)*b,4] if abs(item) > 0]
        #         n_cop=len(cop_count)/((j+1)*b)
        #         n_ftc=len(ftc_count)/((j+1)*b)
        #     else:
        #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color="red",alpha=0.2)
        #         # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
        #         cop_count = [item for item in c_cop[50+j*b:50+(j+1)*b,4] if abs(item) > 0]
        #         ftc_count = [item for item in c_ftc[50+j*b:50+(j+1)*b,4] if abs(item) > 0]
        #         st_cop=len(cop_count)/b
        #         st_ftc=len(ftc_count)/b

        ############### SF, FS #################
        # for j in range(int((len(c_ftc[:,i])-c-ch*b)/d)+ch):
        #     if j<ch:
        #         plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
        #         cop_count = [item for item in c_cop[50:50+(j+1)*b,4] if abs(item) > 0]
        #         ftc_count = [item for item in c_ftc[50:50+(j+1)*b,4] if abs(item) > 0]
        #         n_cop=len(cop_count)/((j+1)*b)
        #         n_ftc=len(ftc_count)/((j+1)*b)
        #     elif ch*b+(j-ch+1)*d<(len(c_ftc[:,i])-c):
        #         plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color=a[j%2],alpha=0.2)
        #         cop_count = [item for item in c_cop[50+ch*b:50+ch*b+(j-ch+1)*d,4] if abs(item) > 0]
        #         ftc_count = [item for item in c_ftc[50+ch*b:50+ch*b+(j-ch+1)*d,4] if abs(item) > 0]
        #         fs_cop=len(cop_count)/((j-ch+1)*d)
        #         fs_ftc=len(ftc_count)/((j-ch+1)*d)
        #     else:
        #         plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color="red",alpha=0.2)
        #         # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
        #         cop_count = [item for item in c_cop[50+ch*b+(j-ch)*d:50+ch*b+(j-ch+1)*d,4] if abs(item) > 0]
        #         ftc_count = [item for item in c_ftc[50+ch*b+(j-ch)*d:50+ch*b+(j-ch+1)*d,4] if abs(item) > 0]
        #         st_cop=len(cop_count)/d
        #         st_ftc=len(ftc_count)/d

        ############### SFS, FSF #################
        for j in range(int((len(c_ftc[:,i])-c-ch*b-(ch2-ch)*d)/b)+ch2):
            if j<ch:
                plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
                cop_count = [item for item in c_cop[50:50+(j+1)*b,4] if abs(item) > 0]
                ftc_count = [item for item in c_ftc[50:50+(j+1)*b,4] if abs(item) > 0]
                n_cop=len(cop_count)/((j+1)*b)
                n_ftc=len(ftc_count)/((j+1)*b)
            elif j<ch2:
                plt.axvspan(xmin=50+ch*b+(j-ch)*d,xmax=50+ch*b+(j-ch+1)*d,color=a[j%2],alpha=0.2)
                cop_count = [item for item in c_cop[50+ch*b:50+ch*b+(j-ch+1)*d,4] if abs(item) > 0]
                ftc_count = [item for item in c_ftc[50+ch*b:50+ch*b+(j-ch+1)*d,4] if abs(item) > 0]
                fs_cop=len(cop_count)/((j-ch+1)*d)
                fs_ftc=len(ftc_count)/((j-ch+1)*d)
            elif ch*b+(ch2-ch)*d+(j-ch2+1)*b<(len(c_ftc[:,i])-c):
                plt.axvspan(xmin=50+ch*b+(ch2-ch)*d+(j-ch2)*b,xmax=50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,color=a[j%2],alpha=0.2)
                cop_count = [item for item in c_cop[50+ch*b+(ch2-ch)*d:50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,4] if abs(item) > 0]
                ftc_count = [item for item in c_ftc[50+ch*b+(ch2-ch)*d:50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,4] if abs(item) > 0]
                sf_cop=len(cop_count)/((j-ch2+1)*b)
                sf_ftc=len(ftc_count)/((j-ch2+1)*b)
            else:
                plt.axvspan(xmin=50+ch*b+(ch2-ch)*d+(j-ch2)*b,xmax=50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,color="red",alpha=0.2)
                # plt.axvspan(xmin=50+j*b,xmax=50+(j+1)*b,color=a[j%2],alpha=0.2)
                cop_count = [item for item in c_cop[50+ch*b+(ch2-ch)*d+(j-ch2)*b:50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,4] if abs(item) > 0]
                ftc_count = [item for item in c_ftc[50+ch*b+(ch2-ch)*d+(j-ch2)*b:50+ch*b+(ch2-ch)*d+(j-ch2+1)*b,4] if abs(item) > 0]
                st_cop=len(cop_count)/b
                st_ftc=len(ftc_count)/b

        plt.axvspan(xmin=50+len(c_ftc[:,i])-c,xmax=len(c_ftc[:,i]),color="green",alpha=0.2)
        cop_count = [item for item in c_cop[len(c_ftc[:,i])-300:len(c_ftc[:,i]),4] if abs(item) > 0]
        ftc_count = [item for item in c_ftc[len(c_ftc[:,i])-300:len(c_ftc[:,i]),4] if abs(item) > 0]
        b_cop=len(cop_count)/300
        b_ftc=len(ftc_count)/300

    plt.show()
    print("n_cop:",n_cop)
    print("n_ftc:",n_ftc)
    print("fs_cop:",fs_cop)
    print("fs_ftc:",fs_ftc)
    print("sf_cop:",sf_cop)
    print("sf_ftc:",sf_ftc)
    print("st_cop:",st_cop)
    print("st_ftc:",st_ftc)
    print("b_cop:",b_cop)
    print("b_ftc:",b_ftc)
