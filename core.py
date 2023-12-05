from kinematics_simulator import nextState
from controller import FeedbackControl
from trajectory_generator import trajectory_generator
import numpy as np 


# def nextState(currentState, controls, speedLimit, dt):
    #return next_state = np.hstack((chassis_next, new_arm_config, new_wheel_angles))

# def FeedbackControl(X, Xd, Xd_next, dt, current_config, Kp=np.eye(6), Ki=np.zeros((6,6)), X_err_sum=np.zeros(6)):
    #return ee_twist, Vd, commanded_speeds, X_err, X_err_sum

# def trajectory_generator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k):
    #return traj_list of the form [Tse, Tsc, Tce, Tce_standoff, Tce_grasp]





def main():
    X_err_sum = np.zeros(6)
    
    
    pass

if __name__ == "__main__":
    main()