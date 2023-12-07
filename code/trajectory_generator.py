"""
Generates a trajectory list for a Kuka YouBot end effector to follow.
The trajectory list is written to a CSV file which can be read by the CoppeliaSim simulator.
Trajectories are calculated using the Modern Robotics library.
"""

import numpy as np 
from modern_robotics import ScrewTrajectory, CartesianTrajectory
import csv

def trajectory_generator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k):
    """
    Generates a trajectory list by calculating intermediary configurations and defining segments.
    
    Parameters:
    - Tse_initial: The initial configuration of the end-effector w.r.t the reference trajectory. 
    - Tsc_initial: The initial configuration of the cube w.r.t the reference trajectory.
    - Tsc_final: The final configuration of the cube w.r.t the reference trajectory.
    - Tce_grasp: The end-effector's configuration w.r.t the cube when grasping it.
    - Tce_standoff: The end-effector's standoff configuration w.r.t the cube.
    - k: The number of trajectory reference configurations per .01 seconds.

    Returns:
    - traj_list: A list of trajectory reference configurations, each represented as a flattened 
    configuration matrix.
    """

    traj_list = []
    Tse_standoff_initial = Tsc_initial @ Tce_standoff
    Tse_grasp_initial = Tsc_initial @ Tce_grasp
    Tse_standoff_final = Tsc_final @ Tce_standoff
    Tse_grasp_final = Tsc_final @ Tce_grasp

    segments = [
        (Tse_initial, Tse_standoff_initial, 'cartesian', int(500/k), 0),
        (Tse_standoff_initial, Tse_grasp_initial, 'screw', int(500/k), 0),
        (Tse_grasp_initial, Tse_grasp_initial, 'cartesian', int(200/k), 1),  # Gripper closes
        (Tse_grasp_initial, Tse_standoff_initial, 'screw', int(500/k), 1),
        (Tse_standoff_initial, Tse_standoff_final, 'screw', int(500/k), 1),
        (Tse_standoff_final, Tse_grasp_final, 'screw', int(500/k), 1),
        (Tse_grasp_final, Tse_grasp_final, 'cartesian', int(200/k), 0),  # Gripper opens
        (Tse_grasp_final, Tse_standoff_final, 'screw', int(500/k), 0),
        (Tse_standoff_final, Tse_standoff_final, 'screw', int(500/k), 0)
    ]

    for (start, end, traj_type, N, gripper_state) in segments:
        if traj_type == 'screw':
            traj = ScrewTrajectory(Xstart=start, Xend=end, Tf=1, N=N, method=5)
        else:  
            traj = CartesianTrajectory(Xstart=start, Xend=end, Tf=1, N=N, method=5)
        traj_list.extend(flatten_configs(traj, gripper_state))
    
    return traj_list


def flatten_configs(configs, gripper_state):
    """
    Flatten list of configurations into 12-vector configurations.
    The gripper state is then added to each configuration, and the
    flattened configurations are returned as a list.
    
    Parameters:
    - configs: A list of configurations (for one chunk of trajectory segments).
    - gripper_state: The gripper state associated with the configurations.

    Returns:
    - flattened_configs: A list of flattened 13-vector configurations.
    """
    configs_array = np.array(configs)  
    
    rotation_matrices = configs_array[:, :3, :3].reshape(-1, 9)  
    position_vectors = configs_array[:, :3, 3]  
    
    gripper_states = np.full((configs_array.shape[0], 1), gripper_state)
    
    flattened_configs = np.hstack((rotation_matrices, position_vectors, gripper_states))
    
    return flattened_configs.tolist()  


def traj_list_to_csv(traj_list):
    """
    Write a trajectory list to a CSV file.

    Parameters:
    - traj_list: A list of trajectory reference configurations.
    """
    with open('traj_list.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(traj_list)
        
def main(): 
    Tsc_initial = np.array([[1,0,0,1], 
                            [0,1,0,0], 
                            [0,0,1,0.025], 
                            [0,0,0,1]]) 
    
    Tsc_goal = np.array([[0,1,0,0], 
                         [-1,0,0,-1], 
                         [0,0,1,0.025], 
                         [0,0,0,1]])

    Tse_initial = np.array([[0,0,1,0],
                            [0,1,0,0],
                            [-1,0,0,0.6],
                            [0,0,0,1]])
    
    Tce_grasp = np.array([[-0.71,0,0.71,0],
                          [0,1,0,0],
                          [-0.71,0,-0.71,0],
                          [0,0,0,1]])
    
    Tce_standoff = np.array([[-0.71,0,0.71,0],
                             [0,1,0,0],
                             [-0.71,0,-0.71,0.35],
                             [0,0,0,1]])
    
    traj_list = trajectory_generator(Tse_initial, Tsc_initial, 
                                     Tsc_goal, Tce_grasp, Tce_standoff, 1)
    traj_list_to_csv(traj_list)
        
    

if __name__ == "__main__": 
    main()