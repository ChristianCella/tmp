# ur5e_mics_safety

This package provides an implementation of HRC safety features for the Merlin's ur5e robot .
The package builds on [this repo by CNR and UniBS](https://github.com/CNR-STIIMA-IRAS/ssm_safety).

It assumes you have a skeleton tracking module that publishes ```geometry_msgs/PoseArray``` on a topic named ```/poses```.
The robot will slow down according to ISO/TS 15066 SSM rules to avoid collisions with the human.

### Test (with simulated robot)

Launch cell:
```
roslaunch ur5e_mics_configurations fake_start.launch pipeline:=ompl
```

Launch safety node:
```
roslaunch ur5e_mics_safety safety.launch
```

Now, you should launch your skeleton tracking module.
As a test, you can publish a msg on the topic ```/poses``` with:

```
rostopic pub -r 10 /poses geometry_msgs/PoseArray "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
poses:
- position:
    x: 0.0
    y: 0.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```

If you try to move the robot towards the pose, the robot should slow down:

<img src="docs/gif_safety_ur5.gif" alt="ur5e_safety" width="70%" height="70%">

### Contact

<img align="center" height="40" src="https://github.com/marco-faroni/marco-faroni.github.io/blob/master/images/marco_circle.png?raw=true"> Marco Faroni
