This program consists of the following files:
- controller.py
- state_transition.py
- trajectory_generator.py 
- kuka_sim.py 

These components come together to enable a youBot mobile manipulator to pick up and place a cube
within the CoppeliaSim environment. 

In trajectory_generator.py, the trajectory for the end effector is created from a series of segments
consisting of desired start and goal configurations. It is a mix of cartesian and screw 
trajectories. 

Odometry is performed as the chassis moves in state_transition.py. The twist commands given are 
generated in controller.py, which uses a feedforward + PI control law. 

Within the results directory, videos can be found for three test cases. The control error plot is 
also provided.
