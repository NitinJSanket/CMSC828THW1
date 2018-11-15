# CMSC828THW1 and CMSC426 (Fall 2018) GTSAM Toy Problem Code
GTSAM Toy Problem <br>
Authors: [Nitin J. Sanket](http://nitinjsanket.github.io) and [Chahat Deep Singh](https://chahatdeep.github.io/).

#### What?
This is a Toy Example to perform SLAM (Simultaneous Localization and Mapping) on a 2D Map. The robot moves around the scene, gets a noisy estimate (distance and direction) of the observed landmarks (you cannot see all landmarks at once) from a simulated LIDAR/Camera sensor. 

This code simulates the environment, robot movement, robot observations and then formulates the SLAM problem as a Factor Graph using the famous [GTSAM](https://borg.cc.gatech.edu/download.html) graph optimization framework.

#### Related Resources
- The accompanying Video lecture for this Toy Problem is can be found [here](https://www.youtube.com/watch?v=o6jEKbnqvTU).
- The accompanying slides can be found [here](https://github.com/NitinJSanket/CMSC828THW1/blob/master/Class10%20GTSAM.pdf).

#### Usage Instructions
- Install GTSAM 3.2.1 from [here](https://borg.cc.gatech.edu/download.html). Try to avoid the latest version of GTSAM  (GTSAM 4.0) as it has some bugs.
- Be sure to be able to run any of the example from the toolbox.
- To run this code, run the `Test1.m` script.

#### Assumptions
The assumptions made are:
- The world extends from -WorldLim to WorldLim in both X and Y directions. Refer to [Test1.m](https://github.com/NitinJSanket/CMSC828THW1/blob/master/Test1.m) script.
- You have a noisy odometry estimate with noise covariance given by [0.3 0.3 0.1] ([MovementX MovementY MovementTheta]).
- The landmark noise is given by [0.1 0.1] in m.
- Landmarks are non blocking (you can see a landmark behind another landmark).
