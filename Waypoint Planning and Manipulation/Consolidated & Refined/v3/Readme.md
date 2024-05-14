To test code:

navigate to directory on 3 terminals:
cd ~/RoboStud2/MyTest/'Waypoint Planning and Manipulation'/'Consolidated & Refined'

Need 3 terminals open in same directory:
1. One terminal to run "roscore"
2. One termminal to run "rosbag play /path/to/your/rosbag.bag"
3. One terminal to run "./genWB"

Remember to run in the order shown above.

When compiling need to run: g++ -o genWB genWB.cpp -I/opt/ros/noetic/include -L/opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization