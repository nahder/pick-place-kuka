
import numpy as np 
def nextState(currentState, controls, speedLimit, dt):
    
    chassis_config = np.array(currentState[0:3]) #current chassis configuration (phi, x, y)
    arm_config = np.array(currentState[3:8]) #current arm configuration (the angles of the 5 arm joints)
    wheel_angles = np.array(currentState[8:12]) #current wheel angles (the angles of the 4 wheels)
    
    wheel_speeds = np.array(controls[0:4])
    arm_joint_speeds = np.array(controls[4:])
    
    #clip the speeds to the speed limit
    wheel_speeds = np.clip(wheel_speeds, -speedLimit, speedLimit)
    arm_joint_speeds = np.clip(arm_joint_speeds, -speedLimit, speedLimit)
    
    # calculate new arm joint angles 
    new_arm_config = arm_config + arm_joint_speeds * dt
    
    # calculate new wheel angles 
    new_wheel_angles = wheel_angles + wheel_speeds * dt
    
    #F = pseudo inverse of H(0), where H(0) is the configuration matrix of the chassis
    
    r = 0.0475
    w = 0.3/2
    l = 0.47/2
    F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                        [1,1,1,1],
                        [-1,1,-1,1]])
    
    body_velocity = F @ wheel_speeds*dt
    
    w_bz = body_velocity[0]
    v_bx = body_velocity[1]
    v_by = body_velocity[2]
    
    if w_bz == 0:
        delta_qb = np.array([0, v_bx, v_by])
    else:
        delta_qb = np.array([w_bz,
                        (v_bx*np.sin(w_bz)+v_by*(np.cos(w_bz)-1))/w_bz,
                        (v_by*np.sin(w_bz)+v_bx*(1-np.cos(w_bz)))/w_bz])
        
    phi_k = chassis_config[0] #previous chassis angle


    #transforming delta_q, the change in the chassis configuration, from the body frame to the space frame
    delta_qs = np.array([[1,0,0],[0,np.cos(phi_k),-np.sin(phi_k)],
                         [0,np.sin(phi_k),np.cos(phi_k)]]) @ delta_qb
    
    # calculate new chassis config from odometry 
    chassis_next = chassis_config + delta_qs
        
    next_state = np.hstack((chassis_next, new_arm_config, new_wheel_angles))
    
    return next_state
    
    
def main():
    current_config = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
    # vel = np.array([10, 10, 10, 10, 0, 0, 0, 0, 0])
    # vel = np.array([-10, 10, -10, 10, 0, 0, 0, 0, 0])
    vel = np.array([-10, 10, 10, -10, 0, 0, 0, 0, 0])

    t = 1
    dt = 0.01
    max_vel = 10
    configs_list = [current_config]
    
    for _ in range(int(t/dt)):
        next_config = nextState(current_config,vel,dt,max_vel)
        current_config = next_config
        configs_list.append(next_config)
        
    print(configs_list[-1])
                
    np.savetxt('configs.csv', configs_list, delimiter=',')
        
        
if __name__ == "__main__":
    main()