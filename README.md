# LBR KDL Dynamics
Short dynamics computation example using KDL.
## Run
Launch a robot
```shell
ros2 launch lbr_bringup lbr_bringup.launch.py sim:=true model:=med7 
```
In another terminal
```shell
ros2 run kdl_inverse_dynamics kdl_inverse_dynamics_node
```
