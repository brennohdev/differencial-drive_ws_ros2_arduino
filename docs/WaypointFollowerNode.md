# WaypointFollowerNode Learning

## 1. The way to get to this point

Otherwise the domain module, the application module related to the Node it was really important to fix some concepts about this ROS2 architecture that I am learning to build.

I learned here that a node is an individual program responsible for one objective task. THat is important because If we want to build a robot we need to keep the code maintanuble and scalabe, The ROS2 architecture comes with this objective -> modularize each responsible to a node. 

## 2. What my node has?

It is a c++ program with the responsibility to follow waypoints through a space. With that in mid, we need to give my Node some steps to fully complete his responsability:

    1. He needs to know where my robots is
    2. He needs to know the target
    3. He needs to point to the target angle and go (applying linear or angular velocity)
    4. And He need to do that most every second because He needs to be autonomus and movel.

And those steps will become parts of my Node, and these parts are pretty important to get to know ROS2 and how it works.

```c++
publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<Odometry>(
        "odom", 10,
        std::bind(&WaypointFollowerNode::odomCallback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        100ms, std::bind(&WaypointFollowerNode::controlLoop, this));
```

As you can see, my class now has a publisher, subscriber and a timer. The publisher creates the channel called ```cmd_vel``` where gives the Twist data typed referencing the linear ang angular velocities.

The subscriber will give the necessary odometry to get directly to the wanted point.

And finally, the timer is our important loop who integrate the PID controller within the two velocities and adjust it for us with the error minimization.


## Note

This is what I understood about what I am learning.





