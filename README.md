# Odometry improvement for Turtlebot3

This package should be used as an overlay to [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) to solve two bugs:

- [wrong odometry](https://github.com/ROBOTIS-GIT/turtlebot3/pull/1001/commits) because of `delta t`
- wheel saturation does not ensure desired curvature
    - this is now done if `max_vel` parameter is passed to the node (classical value is 0.26 m/s)
