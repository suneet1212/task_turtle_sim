# task_turtle_sim
This is a ros package which plans the path of one turtlesim(named turtle2) to go towards the goal point and it tries to avoid turtle1. 

### How to run:
MPC with turtlesim based on unicycle model is in file <a href="https://github.com/suneet1212/task_turtle_sim/blob/main/src/turtle_task.cpp"> turtle_task.cpp </a>


To run the code:
```
roslaunch task_turtle_sim unicycle.launch
```
<br>
MPC with turtlesim based on bicycle model is in file <a href="https://github.com/suneet1212/task_turtle_sim/blob/main/src/task.cpp"> task.cpp </a>


To run the code:

```
roslaunch task_turtle_sim bicycle.launch
```

## Equations Used for Unicycle Model:

x_next = x_current + velocity_current * cos(&theta;_current) * &Delta;T

y_next = y_current + velocity_current * sin(&theta;_current) * &Delta;T

&theta;_next = &theta;_current + angular_velocity_current * &Delta;T

### Objective Function:
(x_n-target_x)<sup>2</sup> + (y_n-target_y)<sup>2</sup> + (theta_n-target_&theta;)<sup>2</sup>

### Constraints:
-1.5 <= velocity[i+1] - velocity[i] <= 1.5      (for i from 0 to N-1)

-1.0 <= angular_velocity[i+1] - angular_velocity[i] <= 1.0      (for i from 0 to N-1)

0 <= x_n <= 10

0 <= y_n <= 10

-3.14 <= &theta;_n <= 3.14


### Constants:
I am initialising the steering_angle as 0, while for position and velocity I initialise them as the current positio and velocity as read by subscribing to the /pose topic

N = MPC Horizon Length

L = biycle model length

&Delta;T is the time period for mpc



## Equations Used for Bicycle Model:

x_next = x_current + velocity_current * cos(steering_angle + &theta;_current) * &Delta;T

y_next = y_current + velocity_current * sin(steering_angle + &theta;_current) * &Delta;T

&theta;_next = &theta;_current + velocity_current * sin(steering_angle) * &Delta;T / L

### Objective Function:
(x_n-target_x)<sup>2</sup> + (y_n-target_y)<sup>2</sup> + (theta_n-target_&theta;)<sup>2</sup>

### Constraints:
-1.5 <= velocity[i+1] - velocity[i] <= 1.5      (for i from 0 to N-1)

-1.5 <= steering_angle[i+1] - steering_angle[i] <= 1.5      (for i from 0 to N-1)

0 <= x_n <= 10

0 <= y_n <= 10

-3.14 <= &theta;_n <= 3.14


### Constants:
I am initialising the steering_angle as 0, while for position and velocity I initialise them as the current positio and velocity as read by subscribing to the /pose topic

N = MPC Horizon Length

L = biycle model length

&Delta;T is the time period for mpc

## Target files:
Run this in a new terminal.(after starting the launch file)
### Unicycle:
```
rosrun task_turtle_sim target
```

### Bicycle:
```
rosrun task_turtle_sim target_bicycle
```

Sometimes there is an error when we run the launch files : "[call_turtle_sim-3] process has died [pid 2832, exit code 2]".
Just do ctrl+c and run again.
