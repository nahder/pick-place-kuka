#inputs:
# A 12-vector representing the current configuration of the robot 
# (3 variables for the chassis configuration, 5 variables for the arm configuration, 
#  and 4 variables for the wheel angles).
# A 9-vector of controls indicating the wheel speeds u {\displaystyle u} (4 variables) and the arm joint speeds θ ˙ {\displaystyle {\dot {\theta }}} (5 variables).
# A timestep Δ t {\displaystyle \Delta t}.
# A positive real value indicating the maximum angular speed of the arm joints and the wheels. For example, if this value is 12.3, 
# the angular speed of the wheels and arm joints is limited to the range [-12.3 radians/s, 12.3 radians/s]. 
# Any speed in the 9-vector of controls that is outside this range will be set to the nearest boundary of the range. 
# If you don't want speed limits, just use a very large number. If you prefer, your function can accept separate speed limits for the wheels and arm joints.

# Output: A 12-vector representing the configuration of the robot time Δ t {\displaystyle \Delta t} later.

# The function NextState is based on a simple first-order Euler step, i.e.,

#     new arm joint angles = (old arm joint angles) + (joint speeds) * Δ t {\displaystyle \Delta t}
#     new wheel angles = (old wheel angles) + (wheel speeds) * Δ t {\displaystyle \Delta t}

def nextState(currentState, controls, dt, speedLimit):
    pass 


def main():
    pass


if __name__ == "__main__":
    main()