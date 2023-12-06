from state_transition import NextState
from controller import FeedbackControl
from trajectory_generator import trajectory_generator
from modern_robotics import FKinBody
import numpy as np 
import matplotlib.pyplot as plt
import logging

def get_traj_idx(traj, idx):
    return np.array([[traj[idx][0], traj[idx][1], traj[idx][2], traj[idx][9]],
                        [traj[idx][3], traj[idx][4], traj[idx][5], traj[idx][10]],
                        [traj[idx][6], traj[idx][7], traj[idx][8], traj[idx][11]],
                        [0, 0, 0, 1]])

def main():
    kp_val = 15.0 
    ki_val = 3.0 
    Kp = np.eye(6)*kp_val
    Ki = np.eye(6)*ki_val

    dt = 0.01
    t = 39
    N = int(t/dt)
    Xerr_i = np.zeros(6)
    err_list = np.zeros((N-1, 6))
    
    T_se_init = np.array([[0, 0, 1, 0],
                     [0, 1, 0, 0],
                     [-1, 0, 0, 0.5],
                     [0, 0, 0, 1]])
    
    T_sc_init = np.array([[1, 0, 0, 1],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0.025],
                     [0, 0, 0, 1]])
    
    Tsc_goal = np.array([[ 0, 1, 0, 0.2],
                        [-1, 0, 0,    -2],
                        [ 0, 0, 1, 0.025],
                        [ 0, 0, 0,     1]])
    
    Tceg = np.array([[ -np.sqrt(2)/2, 0, np.sqrt(2)/2, 0],
                        [ 0, 1, 0, 0],
                        [-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0],
                        [ 0, 0, 0, 1]])

    Tces = np.array([[ 0, 0, 1,   0],
                        [ 0, 1, 0,   0],
                        [-1, 0, 0, 0.2],  
                        [ 0, 0, 0,   1]])
    
    Tb0 = np.array([[1,0,0,0.1662],
                    [0,1,0,0],
                    [0,0,1,0.0026],
                    [0,0,0,1]])

    M0e = np.array([[1,0,0,0.033],
                    [0,1,0,0],
                    [0,0,1,0.6546],
                    [0,0,0,1]])

    Blist = np.array([[0,0,1,0,0.033,0],
                        [0,-1,0,-0.5076,0,0],
                        [0,-1,0,-0.3526,0,0],
                        [0,-1,0,-0.2176,0,0],
                        [0,0,1,0,0,0]]).T

    configs_list = [[0.6, -0.2, -0.2, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0]]
    
    traj = trajectory_generator(T_se_init, T_sc_init, Tsc_goal, Tceg, Tces, 1)
    
    for i in range(N-1):
            current_config = configs_list[-1]            
            phi, x, y = current_config[:3]
            Tsb = np.array([[np.cos(phi), -np.sin(phi),0, x],
                            [np.sin(phi),np.cos(phi),0,y],
                            [0,0,1,0.0963],
                            [0,0,0,1]])
            
            arm_config = current_config[3:8]
            T0e = FKinBody(M0e,Blist,arm_config)
            Tbe = Tb0@T0e
            X = Tsb@Tbe
            
            Xd = get_traj_idx(traj, i)
            Xd_next = get_traj_idx(traj, i+1)

            controls, curr_error = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt,current_config,Xerr_i)            

            next_config = NextState(current_config[:12],controls,dt,10)
            current_config = next_config
            
            next_config = np.hstack((next_config, traj[i][-1]))
            configs_list.append(next_config)
            err_list[i] = curr_error 
            
    np.savetxt("configs_list.csv",configs_list,delimiter=",")
    np.savetxt("err_list.csv",err_list,delimiter=",")
    
    tvec = np.linspace(0,t,N-1)
    plt.plot(tvec,err_list[:,0], label='Xerr_0')
    plt.plot(tvec,err_list[:,1], label='Xerr_1')
    plt.plot(tvec,err_list[:,2], label='Xerr_2')
    plt.plot(tvec,err_list[:,3], label='Xerr_3')
    plt.plot(tvec,err_list[:,4], label='Xerr_4')
    plt.plot(tvec,err_list[:,5], label='Xerr_5')
    plt.title(f'Control error with  kp={kp_val}, ki={ki_val}')
    plt.xlabel('Time (s)')
    plt.ylabel('Error')
    plt.legend()
    plt.savefig(f'Kp={kp_val}_Ki={ki_val}.jpg')
    plt.show()


    
if __name__ == "__main__":
    main()