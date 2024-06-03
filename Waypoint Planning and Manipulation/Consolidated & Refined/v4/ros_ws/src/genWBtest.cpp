// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// // Define Point struct for positions
// struct Point 
// {
//     double x, y, z;
// };

// // Function to convert millimeters to meters
// double mmToMeters(double value) 
// {
//     return value / 1000.0;
// }

// // Function to generate waypoints based on initial and final points
// std::vector<Point> generateWaypoints(const Point& initialWP, const Point& finalWP, int numWaypoints) 
// {
//     std::vector<Point> waypoints;
    
//     // Calculate step size for X, Y, and Z dimensions
//     double stepX = (finalWP.x - initialWP.x) / (numWaypoints + 1);
//     double stepY = (finalWP.y - initialWP.y) / (numWaypoints + 1);
//     double stepZ = (finalWP.z - initialWP.z) / (numWaypoints + 1);
    
//     // Generate equidistant waypoints
//     for (int i = 1; i <= numWaypoints; ++i) 
//     {
//         Point waypoint;
//         waypoint.x = initialWP.x + i * stepX;
//         waypoint.y = initialWP.y + i * stepY;
//         waypoint.z = initialWP.z + i * stepZ;
//         waypoints.push_back(waypoint);
//     }
    
//     return waypoints;
// }

// // Function to publish waypoints as end-effector poses and print the corresponding message
// void publishWaypoints(const std::vector<Point>& waypoints, ros::Publisher& publisher, const std::string& message = "") 
// {
//     for (const auto& waypoint : waypoints) 
//     {
//         geometry_msgs::PoseStamped pose_msg;
//         pose_msg.header.frame_id = "map"; // Assuming waypoints are in map frame
//         pose_msg.header.stamp = ros::Time::now();
//         pose_msg.pose.position.x = waypoint.x;
//         pose_msg.pose.position.y = waypoint.y;
//         pose_msg.pose.position.z = waypoint.z;
//         pose_msg.pose.orientation.x = 1.0; // Retrieve orientation as needed
//         pose_msg.pose.orientation.y = 0.0; // Retrieve orientation as needed
//         pose_msg.pose.orientation.z = 0.0; // Retrieve orientation as needed
//         pose_msg.pose.orientation.w = 0.0; // No rotation

//         publisher.publish(pose_msg);
//         if (!message.empty())
//         {
//             std::cout << message << ": (" << waypoint.x << ", " << waypoint.y << ", " << waypoint.z << "), Orientation: (1, 0, 0, 0)" << std::endl;
//         }
//         ros::Duration(0.5).sleep(); // Sleep for 0.5 seconds between each publication
//     }
// }

// // Function to publish waypoint markers
// void publishWaypointsMarkers(const std::vector<Point>& waypoints, ros::Publisher& publisher) 
// {
//     visualization_msgs::MarkerArray markers;
//     for (size_t i = 0; i < waypoints.size(); ++i) 
//     {
//         // Create marker for waypoint as a red dot
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "map";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "waypoints";
//         marker.id = i; // Unique ID for each marker
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.type = visualization_msgs::Marker::SPHERE;
//         marker.pose.position.x = waypoints[i].x;
//         marker.pose.position.y = waypoints[i].y;
//         marker.pose.position.z = waypoints[i].z;
//         marker.pose.orientation.w = 1.0;
//         marker.scale.x = 0.005; // Diameter of the sphere
//         marker.scale.y = 0.005;
//         marker.scale.z = 0.005;
//         marker.color.r = 1.0;
//         marker.color.a = 1.0;

//         markers.markers.push_back(marker);
//     }

//     publisher.publish(markers);
// }

// // Function to publish box markers underneath waypoints
// void publishBoxMarkers(const std::vector<Point>& waypoints, ros::Publisher& publisher) 
// {
//     visualization_msgs::MarkerArray markers;
//     for (size_t i = 0; i < waypoints.size(); ++i) 
//     {
//         // Create marker for box underneath waypoint
//         visualization_msgs::Marker box;
//         box.header.frame_id = "map";
//         box.header.stamp = ros::Time::now();
//         box.ns = "boxes";
//         box.id = i; // Unique ID for each box
//         box.action = visualization_msgs::Marker::ADD;
//         box.type = visualization_msgs::Marker::CUBE;
//         box.pose.position.x = waypoints[i].x;
//         box.pose.position.y = waypoints[i].y;
//         box.pose.position.z = waypoints[i].z - 0.03; // Offset by 0.03m below waypoint
//         box.pose.orientation.w = 1.0;
//         box.scale.x = 0.025; // Dimensions of the box
//         box.scale.y = 0.025;
//         box.scale.z = 0.025;
//         box.color.g = 1.0; // Green color for the box
//         box.color.a = 1.0;

//         markers.markers.push_back(box);
//     }

//     publisher.publish(markers);
// }

// // Global variables for perception data
// Point perceptionPoint;
// bool perceptionDataReceived = false;

// // ROS callback function for receiving perception data
// void perceptionDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
// {
//     ROS_INFO("Received perception data");

//     // Extract perception data from ROS message array and convert to meters
//     double centX = mmToMeters(msg->data[0]);
//     double centY = mmToMeters(msg->data[1]);
//     double theta = msg->data[2];
//     double length = mmToMeters(msg->data[3]);
//     double width = mmToMeters(msg->data[4]);
//     double height = mmToMeters(msg->data[5]);

//     // Update perceptionPoint with converted data
//     perceptionPoint = {centX, centY, height};
//     perceptionDataReceived = true;
// }

// int main(int argc, char** argv) 
// {
//     // Static Y coordinate
//     //double coodY = 0.3348766372496067;

//     // End-effector offset from box
//     //double offsetZ = (0.7609185611395226 + 0.03);
//     double offsetZ = 0.03;
    
//     // Initialize ROS node
//     ros::init(argc, argv, "waypoint_generator");
//     ros::NodeHandle nh;

//     // Publisher for ROS node to provide end-effector poses
//     ros::Publisher end_effector_pub = nh.advertise<geometry_msgs::PoseStamped>("waypoint_topic", 10);

//     // Publisher for waypoint markers
//     ros::Publisher waypoint_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_markers", 10);

//     // Publisher for box markers
//     ros::Publisher box_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("box_markers", 10);

//     // Subscribe to ROS node providing box dimensions and orientation
//     ros::Subscriber sub = nh.subscribe("/pereption_topic", 1, perceptionDataCallback); // Replace with the actual topic name = perception_topic not pereption_topic (pereption_topic used for testing rosbag of outdated code)

//     // Spin to receive messages until we get the perception data at the desired x-coordinate
//     ros::Rate rate(10); // 10 Hz
//     while (ros::ok() && (!perceptionDataReceived || perceptionPoint.x < 0.01813680100817779)) 
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     if (perceptionDataReceived && perceptionPoint.x >= 0.01813680100817779)
//     {
//         // Define initial and final waypoints using perception data
//         Point initialWP = {perceptionPoint.x, perceptionPoint.y, perceptionPoint.z + offsetZ};
//         Point finalWP = {0.2608303568139733, perceptionPoint.y, perceptionPoint.z + offsetZ}; // Modify with actual final point as necessary

//         // Ensure node is ready and connections are established
//         ros::Duration(1.0).sleep();

//         // Publish initial waypoint
//         std::vector<Point> initialWaypoint = {initialWP};
//         publishWaypoints(initialWaypoint, end_effector_pub, "Initial Waypoint");

//         // Generate waypoints
//         int numWaypoints = 5;
//         std::vector<Point> waypoints = generateWaypoints(initialWP, finalWP, numWaypoints);

//         // Output initial and intermediate waypoints with orientation
//         for (int i = 0; i < waypoints.size(); ++i) 
//         {
//             std::cout << "Waypoint " << i + 1 << ": (" << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].z << "), Orientation: (1, 0, 0, 0)" << std::endl;
//         }

//         // Publish main waypoints
//         publishWaypoints(waypoints, end_effector_pub);

//         // Publish final waypoint
//         std::vector<Point> finalWaypoint = {finalWP};
//         publishWaypoints(finalWaypoint, end_effector_pub, "Final Waypoint");

//         // Add the waypoint that goes down by 0.2m at the final position
//         Point pickUpWP = {finalWP.x, finalWP.y, finalWP.z - offsetZ};

//         // Add the waypoint that goes back up to the final position
//         Point returnWP = {finalWP.x, finalWP.y, finalWP.z};

//         // Define the drop-off point (hardcoded position)
//         Point dropOffWP = {0.3781727542368665, -0.268916276305741, 0.7676042784520107};

//         // Publish additional waypoints (pickup, return, and drop-off)
//         std::vector<Point> additionalWaypoints = {pickUpWP, returnWP, dropOffWP};
//         std::vector<std::string> messages = {"Pick-up Waypoint", "Return to Waypoint", "Drop-off Waypoint"};

//         for (size_t i = 0; i < additionalWaypoints.size(); ++i) 
//         {
//             publishWaypoints({additionalWaypoints[i]}, end_effector_pub, messages[i]);
//         }

//         // Publish waypoints as visualization markers including the initial, pickup and dropoff waypoints 
//         std::vector<Point> allWaypoints = initialWaypoint;
//         allWaypoints.insert(allWaypoints.end(), waypoints.begin(), waypoints.end());
//         allWaypoints.push_back(pickUpWP);
//         allWaypoints.push_back(returnWP);
//         allWaypoints.push_back(dropOffWP);
//         publishWaypointsMarkers(allWaypoints, waypoint_marker_pub);

//         // Add the waypoint markers and the boxes
//         publishWaypointsMarkers(waypoints, waypoint_marker_pub);
//         publishBoxMarkers(waypoints, box_marker_pub);
//     }
//     else
//     {
//         ROS_WARN("Perception data not received or x-coordinate condition not met");
//     }

//     return 0;
// }

#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Define Point struct for positions
struct Point 
{
    double x, y, z;
};

// Define Orientation struct for orientation
struct Orientation
{
    double x, y, z, w;
};

// Function to convert millimeters to meters
double mmToMeters(double value) 
{
    return value / 1000.0;
}

// Function to generate waypoints based on initial and final points
std::vector<Point> generateWaypoints(const Point& initialWP, const Point& finalWP, int numWaypoints) 
{
    std::vector<Point> waypoints;
    
    // Calculate step size for X, Y, and Z dimensions
    double stepX = (finalWP.x - initialWP.x) / (numWaypoints + 1);
    double stepY = (finalWP.y - initialWP.y) / (numWaypoints + 1);
    double stepZ = (finalWP.z - initialWP.z) / (numWaypoints + 1);
    
    // Generate equidistant waypoints
    for (int i = 1; i <= numWaypoints; ++i) 
    {
        Point waypoint;
        waypoint.x = initialWP.x + i * stepX;
        waypoint.y = initialWP.y + i * stepY;
        waypoint.z = initialWP.z + i * stepZ;
        waypoints.push_back(waypoint);
    }
    
    return waypoints;
}

// Function to convert theta to quaternion for rotation around Z-axis
Orientation thetaToQuaternion(double theta)
{
    Orientation quat;
    quat.x = 1.0;
    quat.y = 0.0;
    quat.z = sin(theta / 2.0);
    quat.w = cos(theta / 2.0);
    return quat;
}

// Function to publish waypoints as end-effector poses and print the corresponding message
void publishWaypoints(const std::vector<Point>& waypoints, const Orientation& orientation, ros::Publisher& publisher, const std::string& message = "") 
{
    for (const auto& waypoint : waypoints) 
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map"; // Assuming waypoints are in map frame
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = waypoint.x;
        pose_msg.pose.position.y = waypoint.y;
        pose_msg.pose.position.z = waypoint.z;
        pose_msg.pose.orientation.x = orientation.x;
        pose_msg.pose.orientation.y = orientation.y;
        pose_msg.pose.orientation.z = orientation.z;
        pose_msg.pose.orientation.w = orientation.w;

        publisher.publish(pose_msg);
        if (!message.empty())
        {
            std::cout << message << ": (" << waypoint.x << ", " << waypoint.y << ", " << waypoint.z << "), Orientation: ("
                      << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << ")" << std::endl;
        }
        ros::Duration(0.5).sleep(); // Sleep for 0.5 seconds between each publication
    }
}

// Function to publish waypoint markers
void publishWaypointsMarkers(const std::vector<Point>& waypoints, ros::Publisher& publisher) 
{
    visualization_msgs::MarkerArray markers;
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        // Create marker for waypoint as a red dot
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.id = i; // Unique ID for each marker
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = waypoints[i].x;
        marker.pose.position.y = waypoints[i].y;
        marker.pose.position.z = waypoints[i].z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.005; // Diameter of the sphere
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;
        marker.color.r = 1.0;
        marker.color.a = 1.0;

        markers.markers.push_back(marker);
    }

    publisher.publish(markers);
}

// Function to publish box markers underneath waypoints
void publishBoxMarkers(const std::vector<Point>& waypoints, ros::Publisher& publisher) 
{
    visualization_msgs::MarkerArray markers;
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        // Create marker for box underneath waypoint
        visualization_msgs::Marker box;
        box.header.frame_id = "map";
        box.header.stamp = ros::Time::now();
        box.ns = "boxes";
        box.id = i; // Unique ID for each box
        box.action = visualization_msgs::Marker::ADD;
        box.type = visualization_msgs::Marker::CUBE;
        box.pose.position.x = waypoints[i].x;
        box.pose.position.y = waypoints[i].y;
        box.pose.position.z = waypoints[i].z - 0.03; // Offset by 0.03m below waypoint
        box.pose.orientation.w = 1.0;
        box.scale.x = 0.025; // Dimensions of the box
        box.scale.y = 0.025;
        box.scale.z = 0.025;
        box.color.g = 1.0; // Green color for the box
        box.color.a = 1.0;

        markers.markers.push_back(box);
    }

    publisher.publish(markers);
}

// Global variables for perception data
Point perceptionPoint;
double perceptionTheta;
bool perceptionDataReceived = false;

// ROS callback function for receiving perception data
void perceptionDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ROS_INFO("Received perception data");

    // Extract perception data from ROS message array and convert to meters
    double centX = mmToMeters(msg->data[0]);
    double centY = mmToMeters(msg->data[1]);
    perceptionTheta = msg->data[2];
    double length = mmToMeters(msg->data[3]);
    double width = mmToMeters(msg->data[4]);
    double height = mmToMeters(msg->data[5]);

    // Update perceptionPoint with converted data
    perceptionPoint = {centX, centY, height};
    perceptionDataReceived = true;
}

int main(int argc, char** argv) 
{
    // Static Y coordinate
    //double coodY = 0.3348766372496067;

    // End-effector offset from box
    //double offsetZ = (0.7609185611395226 + 0.03);
    double offsetZ = 0.03;
    
    // Initialize ROS node
    ros::init(argc, argv, "waypoint_generator");
    ros::NodeHandle nh;

    // Publisher for ROS node to provide end-effector poses
    ros::Publisher end_effector_pub = nh.advertise<geometry_msgs::PoseStamped>("waypoint_topic", 10);

    // Publisher for waypoint markers
    ros::Publisher waypoint_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_markers", 10);

    // Publisher for box markers
    ros::Publisher box_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("box_markers", 10);

    // Subscribe to ROS node providing box dimensions and orientation
    ros::Subscriber sub = nh.subscribe("/pereption_topic", 1, perceptionDataCallback); // Replace with the actual topic name

    // Spin to receive messages until we get the perception data at the desired x-coordinate
    ros::Rate rate(10); // 10 Hz
    while (ros::ok() && (!perceptionDataReceived || perceptionPoint.x < 0.01813680100817779)) 
    {
        ros::spinOnce();
        rate.sleep();
    }

    if (perceptionDataReceived && perceptionPoint.x >= 0.01813680100817779)
    {
        // Define initial and final waypoints using perception data
        Point initialWP = {perceptionPoint.x, perceptionPoint.y, perceptionPoint.z + offsetZ};
        Point finalWP = {0.2608303568139733, perceptionPoint.y, perceptionPoint.z + offsetZ}; // Modify with actual final point as necessary

        // Ensure node is ready and connections are established
        ros::Duration(1.0).sleep();

        // Convert theta to quaternion orientation
        Orientation orientation = thetaToQuaternion(perceptionTheta);

        // Publish initial waypoint
        std::vector<Point> initialWaypoint = {initialWP};
        publishWaypoints(initialWaypoint, orientation, end_effector_pub, "Initial Waypoint");

        // Generate waypoints
        int numWaypoints = 5;
        std::vector<Point> waypoints = generateWaypoints(initialWP, finalWP, numWaypoints);

        // Output initial and intermediate waypoints with orientation
        for (int i = 0; i < waypoints.size(); ++i) 
        {
            std::cout << "Waypoint " << i + 1 << ": (" << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].z << "), Orientation: ("
                      << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << ")" << std::endl;
        }

        // Publish main waypoints
        publishWaypoints(waypoints, orientation, end_effector_pub);

        // Publish final waypoint
        std::vector<Point> finalWaypoint = {finalWP};
        publishWaypoints(finalWaypoint, orientation, end_effector_pub, "Final Waypoint");

        // Add the waypoint that goes down by 0.2m at the final position
        Point pickUpWP = {finalWP.x, finalWP.y, finalWP.z - offsetZ};

        // Add the waypoint that goes back up to the final position
        Point returnWP = {finalWP.x, finalWP.y, finalWP.z};

        // Define the drop-off point (hardcoded position)
        Point dropOffWP = {0.3781727542368665, -0.268916276305741, 0.7676042784520107};

        // Publish additional waypoints (pickup, return, and drop-off)
        std::vector<Point> additionalWaypoints = {pickUpWP, returnWP, dropOffWP};
        std::vector<std::string> messages = {"Pick-up Waypoint", "Return to Waypoint", "Drop-off Waypoint"};

        for (size_t i = 0; i < additionalWaypoints.size(); ++i) 
        {
            publishWaypoints({additionalWaypoints[i]}, orientation, end_effector_pub, messages[i]);
        }

        // Publish waypoints as visualization markers including the initial, pickup and dropoff waypoints 
        std::vector<Point> allWaypoints = initialWaypoint;
        allWaypoints.insert(allWaypoints.end(), waypoints.begin(), waypoints.end());
        allWaypoints.push_back(pickUpWP);
        allWaypoints.push_back(returnWP);
        allWaypoints.push_back(dropOffWP);
        publishWaypointsMarkers(allWaypoints, waypoint_marker_pub);

        // Add the waypoint markers and the boxes
        publishWaypointsMarkers(waypoints, waypoint_marker_pub);
        publishBoxMarkers(waypoints, box_marker_pub);
    }
    else
    {
        ROS_WARN("Perception data not received or x-coordinate condition not met");
    }

    return 0;
}
