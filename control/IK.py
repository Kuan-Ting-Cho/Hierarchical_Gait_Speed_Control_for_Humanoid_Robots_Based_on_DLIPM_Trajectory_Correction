import numpy as np
def deg2rad(angle):
    return angle*np.pi/180

def rad2deg(radius):
    return radius/np.pi*180

def traj(x1,x2,x3,x4,x5,x6,foot,R):
    R=np.vstack([np.hstack([R, np.zeros((3, 1))]), [0, 0, 0, 1]])
   #  x1=0
   #  x2=0
   #  x3=-0.4
   #  x4=0.8+2.74*np.pi/180
   #  x5=-0.4-2.74*np.pi/180
   #  x6=0
   #  x3-=0.35
    x4+=2.74*np.pi/180
    x5-=2.74*np.pi/180
    d1, d2, d3= -0.064 ,0.006 ,-0.00725
   #  a2, a3,a4,a5,a6 = 0.091, 0.105, 0.105, 0.069, 0.031 #Johnny
    a2, a3,a4,a5,a6 = 0.102, 0.35795,0.36642,0.029,0.11175 #Roli
    H01 = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,d1],[0,0,0,1]])
    H12 = np.array([[np.sin(x2),0,np.cos(x2),a2*np.sin(x2)],[-np.cos(x2),0,np.sin(x2),-a2*np.cos(x2)],[0,-1,0,d2],[0,0,0,1]])
    H23 = np.array([[np.cos(x3),-np.sin(x3),0,a3*np.cos(x3)],[np.sin(x3),np.cos(x3),0,a3*np.sin(x3)],[0,0,1,d3],[0,0,0,1]])
    H34 = np.array([[np.cos(x4),-np.sin(x4),0,a4*np.cos(x4)],[np.sin(x4),np.cos(x4),0,a4*np.sin(x4)],[0,0,1,0],[0,0,0,1]])
    H45 = np.array([[np.cos(x5),0,np.sin(x5),a5*np.cos(x5)],[np.sin(x5),0,-np.cos(x5),a5*np.sin(x5)],[0,1,0,0],[0,0,0,1]])
    H56 = np.array([[np.cos(x6),-np.sin(x6),0,a6*np.cos(x6)],[np.sin(x6),np.cos(x6),0,a6*np.sin(x6)],[0,0,1,0],[0,0,0,1]])
   #  H6E = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    H6E = R
    
    H5E = np.dot(H56,H6E)
    H4E = np.dot(H45,H5E)
    H3E = np.dot(H34,H4E)
    H2E = np.dot(H23,H3E)
    H1E = np.dot(H12,H2E)
    H0E = np.dot(H01,H1E)
    
   #  H02 = np.dot(H01,H12)
   #  H03 = np.dot(H02,H23)
    # print("H0E =")
    # print(H03)
    xyz=[]
    # for i in range(3): #johnny
    #     xyz.append(H03[i][3])
    # if foot==1: #右腳
    #    xyz[1]=xyz[1]-0.05  #轉成相對於base
    # else:
    #    xyz[1]=xyz[1]+0.05
    for i in range(3): #Roli
       xyz.append(H0E[i][3])
    
    r=(xyz[0]**2+xyz[1]**2)**(1/2)
    xyz[0]=r*np.cos(x1)
    xyz[1]=r*np.sin(x1)
    if foot==1: #右腳
       xyz[1]=xyz[1]-0.105  #轉成相對於base
    else:
       xyz[1]=xyz[1]+0.105
    
   
    return np.array(xyz)