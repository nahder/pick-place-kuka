"""
Feedback control system for a KUKA youBot with an arm mounted to it. The commanded speeds for
the five arm joints and the four wheels are calculated based on the current and desired
end-effector configurations.
"""

import numpy as np 
from modern_robotics import MatrixLog6, TransInv, se3ToVec, Adjoint, FKinBody, JacobianBody

def FeedbackControl(X, Xd, Xd_next,Kp, Ki, dt, current_config, X_err_sum):
    """
    Calculates the commanded speeds for a feedback control system.

    Parameters:
    X: Current end-effector configuration matrix (4x4).
    Xd: Desired end-effector configuration matrix (4x4).
    Xd_next: Next desired end-effector configuration matrix (4x4).
    Kp: Proportional gain (6x6)
    Ki: Integral gain. (6x6)
    dt: Time step. 
    current_config: Current arm configuration vector (5x1).
    X_err_sum: Sum of the end-effector position errors (6x1).

    Returns:
    Commanded speeds for each joint (5x1).
    Updated sum of the end-effector position errors (6x1).
    """
    arm_config = np.array(current_config[3:8]) 
    
    X_err_se3 = (MatrixLog6(TransInv(X) @ Xd)) 
    X_err = se3ToVec(X_err_se3)
    
    Vd_se3 = (MatrixLog6(TransInv(Xd) @ Xd_next)) / dt
    Vd = se3ToVec(Vd_se3)
    
    X_err_sum += X_err * dt
            
    V = Adjoint(TransInv(X) @ Xd) @ Vd + (Kp @ X_err) + (Ki@X_err_sum) 

    Jarm = JacobianBody(Blist, arm_config) 
    T0e = FKinBody(M0e, Blist, arm_config)
    Jbase = Adjoint(TransInv(T0e) @ TransInv(Tb0)) @ F6 
    Je = np.hstack((Jbase, Jarm)) 
    
    commanded_speeds = np.linalg.pinv(Je) @ V
    
    return commanded_speeds, X_err_sum


r = 0.0475
w = 0.3/2
l = 0.47/2
F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                [1,1,1,1],
                [-1,1,-1,1]])

F6 = np.vstack([
    np.zeros(4),
    np.zeros(4),
    F,
    np.zeros(4)
])

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

Xd = np.array([[0, 0, 1, 0.5], 
            [0, 1, 0, 0], 
            [-1, 0, 0, 0.5], 
            [0, 0, 0, 1]])

Xd_next = np.array([[0, 0, 1, 0.6],
                [0, 1, 0, 0], 
                [-1, 0, 0, 0.3],
                [0, 0, 0, 1]])


X = np.array([[0.170, 0, 0.985, 0.387], 
            [0, 1, 0, 0], 
            [-0.985, 0, 0.170, 0.570], 
            [0, 0, 0, 1]])
    
def main():
    
    Kp = np.eye(6)
    Ki = np.zeros((6,6))
    
    config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
    cmds, err = FeedbackControl(X, Xd, Xd_next, Kp, Ki, 0.01, config, np.zeros(6))
    print("cmds: ", cmds)
    print("err: ", err)


if __name__ == "__main__":
    main()
    
    
    
    