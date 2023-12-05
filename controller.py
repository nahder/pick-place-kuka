import numpy as np 
from modern_robotics import MatrixLog6, TransInv, se3ToVec, Adjoint, FKinBody, JacobianBody

r = 0.0475
w = 0.3/2
l = 0.47/2
#F is the pseudo inverse of H(0), where H(0) is the configuration matrix of the chassis
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

def FeedbackControl(X, Xd, Xd_next, dt, current_config, 
                    Kp=np.eye(6), Ki=np.zeros((6,6)), X_err_sum=np.zeros(6)):

    arm_config = np.array(current_config[3:8]) #current arm configuration (the angles of the 5 arm joints)
    
    #error twist, instantaneous error correction command
    X_err_se3 = (MatrixLog6(TransInv(X) @ Xd)) 
    X_err = se3ToVec(X_err_se3)
    
    #twist that takes X to Xd,next in time dt 
    Vd_se3 = (MatrixLog6(TransInv(Xd) @ Xd_next)) / dt
    Vd = se3ToVec(Vd_se3)
    
    X_err_sum += X_err * dt
        
    ee_twist = Adjoint(TransInv(X) @ Xd) @ Vd + (Kp @ X_err) + (Ki@X_err_sum) 
    
    #convert ee twist to command speeds for wheels and arm joints
    #[u,thetadot] = pseudo inverse of mobile manipulator jacobian * ee_twist
    
    Jarm = JacobianBody(Blist, arm_config)  #arm jacobian
    T0e = FKinBody(M0e, Blist, arm_config)
    Jbase = Adjoint(TransInv(T0e) @ TransInv(Tb0)) @ F6 #mobile base jacobian
    Je = np.hstack((Jbase, Jarm)) #mobile manipulator jacobian
    
    commanded_speeds = np.linalg.pinv(Je) @ ee_twist
    
    return ee_twist, Vd, commanded_speeds, X_err, X_err_sum

def main():
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
    
    Kp = np.eye(6)
    Ki = np.zeros((6,6))
    
    config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
    ee_twist, Vd, commanded_speeds, X_err, X_err_sum = FeedbackControl(X, Xd, Xd_next, 
                                                                       0.01, config, Kp, Ki)
    
    print("ee twist: ", ee_twist)
    print("speeds: ", commanded_speeds)
    print("X_err: ", X_err)
    print("Vd: ", Vd)
    print("X_err_sum: ", X_err_sum)
    
if __name__ == "__main__":
    main()
    
    
    
    