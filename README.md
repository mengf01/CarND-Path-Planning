# Outline of Path-Planning-Project

Here I finished the CarND (Self-Driving Car Engineer Nanodegree Program) project on path planning and you can find the details of the project requirement here: ([https://github.com/udacity/CarND-Path-Planning-Project](https://github.com/udacity/CarND-Path-Planning-Project))
  
## Implementation details

### Baseline: stay in lane with reasonable velocity, acceleration and jerk

Following the suggestion in Q%A session of the project, I did the following:

1, In order to have a smooth transition from last step, I kept all the path points planned in the previous step, and add more (see 2 below) to make its length equal 50.

2, To plan path at the current step, I used the previous path's end point as the starting reference. I created a list of waypoints that are widely spaced (~30m). I used them to do spline fitting and then interpolation to get the needed path points. 

3, The interpolation above gives path points, the spacing of which is determined by the velocity `ref_vel`. Then they are appended to the path of the previous step as the new path. The interpolation process in car's local frame.

### Collision avoidance
I used sensor fusion data to loop through all the actors. If there is an actor within the lane that I'm targeting and it's within 30m ahead of the SDV, I will decrease the velocity `ref_vel` by 0.22 at this step. Otherwise, I would increase `ref_vel` by 0.22 if we are below the speed limit. A flag `too_close` has been set for this.

### Lane change
It's appealing to change lane when there is a car ahead of us and it's blocking us. Here I'd like to change lane if meeting all the following criteria:

1, `ref_vel` > `LANE_CHANGE_VEL`, so that the car can swiftly shift lane;

2, there is at least AHEAD_BUFFER (meters ahead of the ref point) in the available lane;

3, there is at least BEHIND_BUFFER (meters behind current car position) in the available lane;

4, If more than 1 lane are available, choose the lane that the car ahead is farther from us.

SDV should not change 2 lanes at one time or change our mind after decided to change lane. Thus, I used a variable `cool_down`, which will decrease by 1 each time (until 0), and reset to maximum whenever SDV starts to change lane. If SDV is currently changing lane (i.e., `cool_down` has not decreased to 0), the SDV would not plan a new lane change.

Lane change will happen at constant speed.
