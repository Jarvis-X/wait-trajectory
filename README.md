# A mini trajctory generator that waits for the robot.

## Initialization:
- choose onr type of trajectory you want to use:
  - If you want to define the trajectory by defining the linear/angular position, velocity, acceleration (optional) in each axis explicitly as a function of time
  ```python
  planner_eq = TrajectoryPlanner3D('equations',
                                    pos_func=pos_func,
                                    vel_func=vel_func,
                                    angle_func=angle_func,
                                    omega_func=omega_func)
  ```
  - If you want to define the trajectory using waypoints of linear/angular position, velocity, acceleration (optional) in each axis at certain time stamps and then smoothify the paths using Quintic polynomials
  ```python
  waypoints = [
      (np.array([0,0,0]), np.array([0.5,0,0]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,np.pi/2]), np.array([0,0,0]), 0),
      (np.array([1,2,1]), np.array([0.2,0.1,0]), np.array([0,0,0]), np.array([0,0,np.pi/2]), np.array([0,0,np.pi]), np.array([0,0,0]), 2),
      (np.array([2,0,2]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,np.pi]), np.array([0,0,0]), np.array([0,0,0]), 4)
  ] # (pos, vel, acc, angles, omega, alpha, time)

  planner_poly = TrajectoryPlanner3D('quintic_polynomial', waypoints=waypoints, time_scaling_gain=0.8)
  ```

  ## Obtain all the waypoints of the trajectory in a period of time at a fixed time increment
  ```python
  trajctory_waypoints = planner.warm_start(dt=0.1, total_time=10) # list of [x, v, a, theta, omega, alpha]
  planner.animate(save=False) # only call this animate function after a warm_start
  ```

  ## Get the desired trajectory waypoint at time t, given the current robot position.
  - The adaptive time scaling will make the planner give a desired waypoint that is slowed down/sped up for the current robot position.
  - If `robot_pos = None`, then the raw desired waypoint at t is returned without scaling.
  ```python
  planner.get(t, robot_pos=robot_pos)
  ```
