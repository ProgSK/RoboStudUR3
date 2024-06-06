# RoboStudUR3

Git repository for UR3 Pick and Place robot for Robotics Studio 2 - Group 17

#################################################################
Perception Progress Log:
#################################################################

#################################################################
Waypoint Planning & Manipulation Progress Log:
#################################################################

22/04/2024 - Updated github for all the subsystem test files for waypoint planning and manipulation for all code files worked on from week 2 - week 7.

04/05/2024 - Created a consolidated singular file called genWB.cpp to combine all genWay, genBoxWay and genRealWay.cpp files together. Also accounted for subscription and retrieval of perception data from ROS nodes. Still need to refine visualisation for RViz. Still need to test on ROS and RViz.

02/06/2024:
Spent time since end of semester and time in labs for following last updates to waypoint code:

Created v3 code for integrating waypoint<-->movement/collision code
- Is an entirely different code file with same code by consolidating v1 and v2 code files into new v3 code file
- Has no integration with Perception due to issues with using perception data with movement/collision part of system
- Worst-case scenario code for demo
- RViz works and displays waypoints and box markers

Created v4 code for complete system integration: perception<-->waypoint<-->movement/collision
- Is an entirely different code file from v3
- Has integration with Perception
- Best-case scenario code for demo
- RViz works and displays waypoints and box markers

#################################################################
Trajectory Planning & Control + Collision Detection & Avoidance Code
#################################################################
Google drive link to the code for Trajectory Planning & Control + Collision Detection & Avoidance:
https://drive.google.com/drive/folders/1AEyNizdOIOfVngCimG3Y0oDE7cMPWiLK?usp=sharing

#################################################################
Trajectory Planning & Control Progress Log:
#################################################################

#################################################################
Collision Detection & Avoidance Progress Log:
#################################################################
