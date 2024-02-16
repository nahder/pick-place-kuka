from state_transition import NextState
from controller import FeedbackControl
from trajectory_generator import trajectory_generator
from modern_robotics import FKinBody
import numpy as np 
import matplotlib.pyplot as plt

def get_traj_idx(traj, idx):
    """
    Get the transformation matrix at a specific index from a trajectory.

    Parameters:
    traj (list): The trajectory containing transformation matrices.
    idx (int): The index of the desired transformation matrix.

    Returns:
    numpy.ndarray: The transformation matrix at the specified index.
    """
    return np.array([[traj[idx][0], traj[idx][1], traj[idx][2], traj[idx][9]],
                        [traj[idx][3], traj[idx][4], traj[idx][5], traj[idx][10]],
                        [traj[idx][6], traj[idx][7], traj[idx][8], traj[idx][11]],
                        [0, 0, 0, 1]])

def main():
    """
    Main function that executes the Kuka simulation.

    This function performs the following steps:
    1. Initializes the control gains (Kp and Ki).
    2. Sets the simulation parameters (dt, t, N).
    3. Initializes the error variables (err_sum, err_list).
    4. Sets the initial transformation matrices (T_se_init, T_sc_init).
    5. Sets the goal transformation matrix (Tsc_goal).
    6. Sets the transformation matrices for end-effector and goal frames (Tceg, Tces).
    7. Sets the base and end-effector transformation matrices (Tb0, M0e).
    8. Sets the joint configuration list (configs_list).
    9. Generates the trajectory using the trajectory_generator function.
    10. Performs the simulation loop for each time step:
        a. Computes the current configuration of the robot.
        b. Computes the desired end-effector transformation (Xd) and its next value (Xd_next).
        c. Computes the control inputs and current error using the FeedbackControl function.
        d. Computes the next configuration of the robot using the NextState function.
        e. Appends the next configuration to the configuration list.
        f. Updates the current error in the error list.
    11. Saves the configuration and error lists to CSV files.
    12. Plots the control error over time.
    """
    
    print("Initializing system parameters...") 
    kp_val = 2.0
    ki_val = 0.5
    Kp = np.eye(6)*kp_val
    Ki = np.eye(6)*ki_val

    dt = 0.01
    t = 39.0
    N = int(t/dt)
    err_sum = np.zeros(6)
    err_list = np.zeros((N-1, 6))
    
    T_se_init = np.array([[0, 0, 1, 0],
                     [0, 1, 0, 0],
                     [-1, 0, 0, 0.5],
                     [0, 0, 0, 1]])

    # T_sc_init = np.array([[1, 0, 0, 1],
    #                  [0, 1, 0, 0],
    #                  [0, 0, 1, 0.025],
    #                  [0, 0, 0, 1]])
    
    # Tsc_goal = np.array([[ 0, 1, 0, 0.0],
    #                     [-1, 0, 0,    -1],
    #                     [ 0, 0, 1, 0.025],
    #                     [ 0, 0, 0,     1]])
    #new task configs
    T_sc_init = np.array([[1, 0, 0, 1],
                     [0, 1, 0, 1],
                     [0, 0, 1, 0.025],
                     [0, 0, 0, 1]])
    
    Tsc_goal = np.array([[ 0, 1, 0, -1],
                        [-1, 0, 0,    -1],
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

    configs_list = [[0.55, 0.2, 0.2, 0, 0, 0.0, -1.5, 0, 0, 0, 0, 0, 0]]
    
    print("Generating trajectory based on initial and goal configurations...")
    traj = trajectory_generator(T_se_init, T_sc_init, Tsc_goal, Tceg, Tces, 1)
    
    for i in range(N-1):
            current_config = configs_list[-1]            
            phi, x, y = current_config[:3]
            Tsb = np.array([[np.cos(phi), -np.sin(phi),0, x],
                            [np.sin(phi),np.cos(phi),0,y],
                            [0,0,1,0.0963],
                            [0,0,0,1]])
            
            arm_config = current_config[3:8]
            T0_e = FKinBody(M0e,Blist,arm_config)
            Tbe = Tb0@T0_e
            X = Tsb@Tbe
            
            Xd = get_traj_idx(traj, i)
            Xd_next = get_traj_idx(traj, i+1)
            if i % 500 == 0:
                print(f"Generating command twist for time step {i}") 
                
            controls, curr_error = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt,current_config,err_sum)            
            next_config = NextState(current_config[:12],controls,dt,50)
            current_config = next_config
            
            next_config = np.hstack((next_config, traj[i][-1]))
            configs_list.append(next_config)
            err_list[i] = curr_error 
            

    tvec = np.linspace(0,10,N-1)
    
    print("Generating a plot of the control error...")
    plt.plot(tvec,err_list[:,0], label='roll')
    plt.plot(tvec,err_list[:,1], label='pitch')
    plt.plot(tvec,err_list[:,2], label='yaw')
    plt.plot(tvec,err_list[:,3], label='x')
    plt.plot(tvec,err_list[:,4], label='y')
    plt.plot(tvec,err_list[:,5], label='z')
    plt.title(f'Control error with kp={kp_val}, ki={ki_val}')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m/s or rad/s)')
    print("Storing the configurations and errors in CSV files...")
    plt.legend()
    plt.show()
    np.savetxt("/results/newTask/configs_list.csv",configs_list,delimiter=",")
    np.savetxt("/results/newTask/err_list.csv",err_list,delimiter=",")
    print("Program complete.")

if __name__ == "__main__":
    main()