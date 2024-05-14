#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

/* ORIGINAL CODE */

// Define Point struct for positions
struct Point 
{
    double x, y, z;
};

// Forward declaration of visualizeWaypointsAndBox
void visualizeWaypointsAndBox(const Point& initialWP, const std::vector<Point>& waypoints, double length, double width, double height);

// Function to calculate distance between two points (only X coordinate)
double distanceX(const Point& p1, const Point& p2) 
{
    return std::abs(p2.x - p1.x);
}

// Global variables for temporary z value and flag for z update
double tempZ = 5.0;
bool zUpdated = false;

// Function to generate waypoints based on initial and final points
std::vector<Point> generateWaypoints(const Point& initialWP, const Point& finalWP, int numWaypoints) 
{
    std::vector<Point> waypoints;
    
    // Calculate step size for X dimension
    double stepX = (finalWP.x - initialWP.x) / (numWaypoints + 1);
    
    // Generate equidistant waypoints
    for (int i = 1; i <= numWaypoints; ++i) 
    {
        Point waypoint;
        waypoint.x = initialWP.x + i * stepX;
        waypoint.y = initialWP.y;
        waypoint.z = initialWP.z;
        waypoints.push_back(waypoint);
    }
    
    return waypoints;
}

// ROS callback function for receiving perception data
void perceptionDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ROS_INFO("Received perception data");

    // Extract perception data from ROS message array
    double centX = msg->data[0];
    double centY = msg->data[1];
    double theta = msg->data[2];
    double length = msg->data[3];
    double width = msg->data[4];
    double height = msg->data[5];

    // If z coordinate is updated, set zUpdated flag and update tempZ
    if (!zUpdated) 
    {
        tempZ = height/2; // Assuming the mid-point of the height of the box will be the same z-coordinate as for waypoints
        zUpdated = true;
    }
    
    // Generate waypoints based on received data
    Point initialWP = {centX, centY, tempZ}; // Use tempZ as z-coordinate based on above zUpdated boolean check
    Point finalWP = {50.0, centY, tempZ}; // Hard-coded final position for x-coordinate
    int numWaypoints = 12;
    std::vector<Point> waypoints = generateWaypoints(initialWP, finalWP, numWaypoints);
    
    // Output information for each waypoint
    std::cout << "\nGenerated Waypoints:" << std::endl;
    std::cout << "Distance between Initial Waypoint to Waypoint 1: " << distanceX(initialWP, waypoints[0]) << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        std::cout << "Waypoint " << i + 1 << ": (" << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].z << ")" << std::endl;
        
        // Calculate difference in x-coordinate between waypoints
        if (i > 0) 
        {
            double distX = distanceX(waypoints[i - 1], waypoints[i]);
            std::cout << "Distance between Waypoint " << i << " to Waypoint " << i + 1 << ": " << distX << std::endl;
        }
    }
    std::cout << "Distance between Waypoint " << numWaypoints << " to Final Waypoint: " << distanceX(waypoints.back(), finalWP) << std::endl;
    std::cout << "Total waypoints generated: " << waypoints.size() << std::endl;

    // Visualize waypoints and box prism
    visualizeWaypointsAndBox(initialWP, waypoints, length, width, height);
}

// Function to visualize waypoints and box prism using RViz
void visualizeWaypointsAndBox(const Point& initialWP, const std::vector<Point>& waypoints, double length, double width, double height)
{
    // Create a ROS node handle
    ros::NodeHandle nh;

    // Publisher for visualization markers
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    // Create a marker array
    visualization_msgs::MarkerArray marker_array;

    // Create a marker for waypoints
    visualization_msgs::Marker waypoints_marker;
    waypoints_marker.header.frame_id = "map"; // Set the frame ID
    waypoints_marker.header.stamp = ros::Time::now();
    waypoints_marker.ns = "waypoints";
    waypoints_marker.id = 0; // Unique ID for this marker
    waypoints_marker.type = visualization_msgs::Marker::POINTS;
    waypoints_marker.action = visualization_msgs::Marker::ADD;
    waypoints_marker.pose.orientation.w = 1.0;
    waypoints_marker.scale.x = 0.2;
    waypoints_marker.scale.y = 0.2;
    waypoints_marker.color.r = 1.0;
    waypoints_marker.color.a = 1.0;

    // Add waypoints to marker
    for (const auto& waypoint : waypoints) 
    {
        geometry_msgs::Point p;
        p.x = waypoint.x;
        p.y = waypoint.y;
        p.z = waypoint.z;
        waypoints_marker.points.push_back(p);
    }
    marker_array.markers.push_back(waypoints_marker);

    // Create a marker for the box prism
    visualization_msgs::Marker box_marker;
    box_marker.header.frame_id = "map"; // Set the frame ID
    box_marker.header.stamp = ros::Time::now();
    box_marker.ns = "box";
    box_marker.id = 1; // Unique ID for this marker
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.pose.position.x = initialWP.x;
    box_marker.pose.position.y = initialWP.y;
    box_marker.pose.position.z = initialWP.z + height / 2; // Adjust for center of the box
    box_marker.pose.orientation.w = 1.0;
    box_marker.scale.x = length;
    box_marker.scale.y = width;
    box_marker.scale.z = height;
    box_marker.color.r = 0.0;
    box_marker.color.g = 1.0;
    box_marker.color.b = 0.0;
    box_marker.color.a = 0.5;

    marker_array.markers.push_back(box_marker);

    // Publish marker array
    marker_pub.publish(marker_array);
} 

int main(int argc, char** argv) 
{
    // Initialize ROS node
    ros::init(argc, argv, "waypoint_topic");
    ros::NodeHandle nh;

    // Subscribe to ROS node providing box dimensions and orientation
    ros::Subscriber sub = nh.subscribe("/perception_topic", 1, perceptionDataCallback); // Remember to replace "/perception_data" with the actual topic name from dominic's ROS node that publishes the box pose information

    // Spin to receive messages
    ros::spin();

    return 0;
}

/* TESTING CODE

void visualizeWaypointsAndBox();

int main(int argc, char** argv) 
{
    // Initialize ROS node
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh;

    // Publish visualization markers
    visualizeWaypointsAndBox();

    return 0;
}

void visualizeWaypointsAndBox()
{
    // Create a ROS node handle
    ros::NodeHandle nh;

    // Publisher for visualization markers
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    // Create a marker array
    visualization_msgs::MarkerArray marker_array;

    // Create a marker for waypoints
    visualization_msgs::Marker waypoints_marker;
    waypoints_marker.header.frame_id = "map"; // Set the frame ID
    waypoints_marker.header.stamp = ros::Time::now();
    waypoints_marker.ns = "waypoints";
    waypoints_marker.id = 0; // Unique ID for this marker
    waypoints_marker.type = visualization_msgs::Marker::POINTS;
    waypoints_marker.action = visualization_msgs::Marker::ADD;
    waypoints_marker.pose.orientation.w = 1.0;
    waypoints_marker.scale.x = 0.2;
    waypoints_marker.scale.y = 0.2;
    waypoints_marker.color.r = 1.0;
    waypoints_marker.color.a = 1.0;

    // Add waypoints to marker
    geometry_msgs::Point p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    waypoints_marker.points.push_back(p1);

    geometry_msgs::Point p2;
    p2.x = 1;
    p2.y = 1;
    p2.z = 1;
    waypoints_marker.points.push_back(p2);

    marker_array.markers.push_back(waypoints_marker);

    // Create a marker for the box prism
    visualization_msgs::Marker box_marker;
    box_marker.header.frame_id = "map"; // Set the frame ID
    box_marker.header.stamp = ros::Time::now();
    box_marker.ns = "box";
    box_marker.id = 1; // Unique ID for this marker
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.pose.position.x = 0;
    box_marker.pose.position.y = 0;
    box_marker.pose.position.z = 0;
    box_marker.pose.orientation.w = 1.0;
    box_marker.scale.x = 1;
    box_marker.scale.y = 1;
    box_marker.scale.z = 1;
    box_marker.color.r = 0.0;
    box_marker.color.g = 1.0;
    box_marker.color.b = 0.0;
    box_marker.color.a = 0.5;

    marker_array.markers.push_back(box_marker);

    // Publish marker array
    marker_pub.publish(marker_array);
}

*/