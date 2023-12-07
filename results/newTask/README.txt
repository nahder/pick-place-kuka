The Kp gain is 2.0 and Ki gain is 0.5. This is using a feedforward + PI controller. 
The robot was successful at picking up the block and placing it at the desired position.
There is no overshoot and the error twist decays rapidly. This pick and place operation uses a 
new poses at the beginning and end for the cube. It begins at (-1,-1) and the robot places it 
at (1,1). The robot also begins in a configuration that is displaced from the others by 0.2 meters
and is in a orientation 30 degrees off. 
