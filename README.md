This project has several components which come together to enable a KUKA mobile manipulator to pick up and place a cube within the CoppeliaSim environment.


The <code>NextState</code> function, implemented in <a href="https://github.com/nahder/pick-place-kuka/blob/main/code/state_transition.py" target="_blank"><code>state_transition.py</code></a>, takes the following inputs:

<div align="center">
    <img src="/assets/kuka_error.png" alt="Results" width="60%" />
    <br>
</div>

<div align="center">
    <table style="border-collapse: separate; width: 50%; text-align: left; border-spacing: 0;">
        <tr>
            <th style="border: 0.75px solid gray; padding: 3.5px; background-color: #305b40; border-top-left-radius: 10px;">Parameter</th>
            <th style="border: 0.75px solid gray; padding: 3.5px; background-color: #305b40; border-top-right-radius: 10px;">Description</th>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>currentState</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">12-vector representing the current robot configuration (3 for the chassis configuration, 5 for the arm configuration, and 4 for the wheel angles)</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>controls</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">9-vector of controls (4 for the wheel speeds, 5 for the arm joint speeds)</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>dt</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">Timestep</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px; border-bottom-left-radius: 10px;"><code>speedlimit</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px; border-bottom-right-radius: 10px;">Maximum speed limit for the youBot</td>
        </tr>
    </table>
</div>




and outputs a new 12-vector configuration after time `dt` has passed. The new arm joint and wheel angles are computed using a first-order Euler step, while the chassis configuration is updated using odometry estimates for a four-mecanum-wheel robot. 

### II. Trajectory Generation
The planner for the youBot end effector is implemented in <a href="https://github.com/nahder/pick-place-kuka/blob/main/code/trajectory_generator.py" target="_blank"><code>trajectory_generator.py</code></a>. It takes the following inputs:

<div align="center">
    <table style="border-collapse: separate; width: 50%; text-align: left; border-spacing: 0;">
        <tr>
            <th style="border: 0.75px solid gray; padding: 3.5px; background-color: #305b40; border-top-left-radius: 10px;">Parameter</th>
            <th style="border: 0.75px solid gray; padding: 3.5px; background-color: #305b40; border-top-right-radius: 10px;">Description</th>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>Tse_init</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">The initial end effector configuration</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>Tsc_init</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">The cube's initial configuration</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>Tsc_final</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">The cube's desired final configuration</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>Tsc_final</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">The end-effector configuration relative to the cube when grasping</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>Tce_standoff</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">The end effector's standoff configuration above the cube</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px; border-bottom-left-radius: 10px;"><code>k</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px; border-bottom-right-radius: 10px;">The number of trajectory reference configurations per 0.01 seconds</td>
        </tr>
    </table>
</div>


and outputs a list of flattened reference trajectories for inputting into the CoppeliaSim simulator. These trajectories are a mix of screw and Cartesian types generated with the help of the <a href="https://github.com/NxRLab/ModernRobotics" target="_blank"><code>modern_robotics</code></a> library.



### III. Feedforward Control
The controller, implemented in <a href="https://github.com/nahder/pick-place-kuka/blob/main/code/controller.py" target="_blank"><code>controller.py</code></a>, is based on a feedforward plus feedback control law.

<div align="center">
    <img src="/assets/31n.svg" alt="Control Law Equation" width="60%" />
    <br>
</div>

The terms are:

<div align="center">
    <table style="border-collapse: separate; width: 50%; text-align: left; border-spacing: 0;">
        <tr>
            <th style="border: 0.75px solid gray; padding: 3.5px; background-color: #305b40; border-top-left-radius: 10px;">Parameter</th>
            <th style="border: 0.75px solid gray; padding: 3.5px; background-color: #305b40; border-top-right-radius: 10px;">Description</th>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>X</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">The current actual end effector configuration</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>Xd</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">The current end effector reference configuration</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>Xerr</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">The error twist</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>Kp and Ki</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">PI gain matrices</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px;"><code>V(d)</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px;">The feedforward reference twist</td>
        </tr>
        <tr>
            <td style="border: 0.75px solid gray; padding: 3.5px; border-bottom-left-radius: 10px;"><code>V(t)</code></td>
            <td style="border: 0.75px solid gray; padding: 3.5px; border-bottom-right-radius: 10px;">The commanded end effector twist</td>
        </tr>
    </table>
</div>



### IV. Results 

The robot was successful at picking up the block and placing it at the desired position.
There is no overshoot and the error twist decays rapidly.

<div align="center">
    <img src="/assets/kuka_error.png" alt="Results" width="60%" />
    <br>
</div>
<br>
