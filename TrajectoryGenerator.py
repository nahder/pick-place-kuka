"""
TrajectoryGenerator to generate the reference trajectory for the end-effector frame {e}. 
This trajectory consists of eight concatenated trajectory segments, as described above. 
Each trajectory segment begins and ends at rest. Below are suggested inputs and outputs; you may modify these if you wish. 
"""
import numpy as np 
import modern_robotics as mr 


def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k): 
    """
    Inputs:
    Tse_initial: end effector initial config
    Tsc_initial: cube initial config
    Tsc_final: cube final config
    
    Tce_grasp: end effector config wrt cube when grasping
    Tce_standoff: end effector config wrt cube in standoff pos 
    k: The number of trajectory reference configurations per 0.01 seconds,

    Output:
    traj: The concatenated matrix of reference configurations, where each is expressed in the end-effector frame {e}.
    """
    
    traj_list = []  
    
    # 1: Move from initial ee position to standoff 
    
    # 2: Move from standoff to grasp position 
    
    # 3: Close the gripper 
    
    # 4: Move from grasp position to standoff (going back up)
    
    # 5: Move ee to standoff above the final position 
    
    # 6: Move ee to final position 
    
    # 7: Open the gripper
    
    # 8: Move ee to standoff above the final position
    
    # 9: Return to original ee position 
    
    

def main(): 
    pass 

if __name__ == "__main__": 
    main()