# Rapidly Exploring Random Tree Robot Planning
Using Python TK to visualize, point/line robots generate planning trees in their
configuration space to maneuvre to the goal point. 

![uni](https://i.imgur.com/3UX3cUw.png "Using a uniform distribution to sample points")
![gauss](https://i.imgur.com/s02Kzig.png "Using a Gaussian distribution to sample points")
![line](https://i.imgur.com/PMMXtQX.png "Line robot of length 50px")

## Usage
Run `./rrt_planner_point_robot.py <image>` to generate a world based on the image given
and an expanding tree towards the goal.
