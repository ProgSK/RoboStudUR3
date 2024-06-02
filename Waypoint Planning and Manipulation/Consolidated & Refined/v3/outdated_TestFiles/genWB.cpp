#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

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
bool dataFrozen = false; // Flag to check if data is frozen
bool dataReceived = false; // Flag to check if any data has been received
bool waypointsGenerated = false; // Flag to check if waypoints are already generated
double endX = -0.1923570937198585; // End point

// Conversion function from millimeters to meters
void convertToMeters(double& centx, double& centy, double& length, double& width, double& height) 
{
    centx /= 1000.0;
    centy /= 1000.0;
    length /= 1000.0;
    width /= 1000.0;
    height /= 1000.0;
}

// Function to reverse the direction of perception data
void reverseDirection(double& centX, double& centY)
{
    centX = -centX;
    centY = -centY;
}

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

// Global storage for latest perception data
Point latestWP;
double latestTheta, latestLength, latestWidth, latestHeight;

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

    // Reverse the direction of the perception data
    reverseDirection(centX, centY);

    // Convert to meters
    convertToMeters(centX, centY, length, width, height);

    // Update the latest perception data
    latestWP = {centX, centY, height / 2};
    latestTheta = theta;
    latestLength = length;
    latestWidth = width;
    latestHeight = height;
    dataReceived = true;
}

// Function to publish waypoints and orientation
void publishWaypoints(ros::Publisher waypoints_pub, ros::Publisher orientation_pub)
{
    if (dataReceived && !waypointsGenerated) 
    {
        Point finalWP = {endX, latestWP.y, latestWP.z};
        int numWaypoints = 5;
        std::vector<Point> waypoints = generateWaypoints(latestWP, finalWP, numWaypoints);

        // Publish waypoints
        geometry_msgs::PoseArray waypoints_msg;
        waypoints_msg.header.frame_id = "map";
        waypoints_msg.header.stamp = ros::Time::now();

        for (const auto& waypoint : waypoints) 
        {
            geometry_msgs::Pose pose;
            pose.position.x = waypoint.x;
            pose.position.y = waypoint.y;
            pose.position.z = waypoint.z;
            waypoints_msg.poses.push_back(pose);
        }

        waypoints_pub.publish(waypoints_msg);

        // Publish orientation
        geometry_msgs::PoseStamped orientation_msg;
        orientation_msg.header.frame_id = "map";
        orientation_msg.header.stamp = ros::Time::now();
        orientation_msg.pose.orientation.x = 0.0;
        orientation_msg.pose.orientation.y = 0.0;
        orientation_msg.pose.orientation.z = sin(latestTheta / 2);
        orientation_msg.pose.orientation.w = cos(latestTheta / 2);

        orientation_pub.publish(orientation_msg);

        std::cout << "\nGenerated Waypoints:" << std::endl;
        std::cout << "Initial Waypoint: (" << latestWP.x << ", " << latestWP.y << ", " << latestWP.z << ")" << std::endl;
        std::cout << "Distance between Initial Waypoint to Waypoint 1: " << distanceX(latestWP, waypoints[0]) << std::endl;
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
        std::cout << "Final Waypoint: (" << finalWP.x << ", " << finalWP.y << ", " << finalWP.z << ")" << std::endl;
        std::cout << "Distance between Waypoint " << numWaypoints << " to Final Waypoint: " << distanceX(waypoints.back(), finalWP) << std::endl;
        std::cout << "Total waypoints generated: " << waypoints.size() << std::endl;

        // Visualize waypoints and box prism
        visualizeWaypointsAndBox(latestWP, waypoints, latestLength, latestWidth, latestHeight);

        // Set flag to true to prevent generating waypoints again
        waypointsGenerated = true;
    } else {
        //ROS_WARN("No perception data received yet. Unable to generate waypoints.");
    }
}

// Function to visualize waypoints and box prism using RViz
void visualizeWaypointsAndBox(const Point& initialWP, const std::vector<Point>& waypoints, double length, double width, double height)
{
    // Create a ROS node handle
    ros::NodeHandle nh;

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
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    marker_pub.publish(marker_array);
}

int main(int argc, char** argv) 
{
    // Initialize ROS node
    ros::init(argc, argv, "waypoint_topic");
    ros::NodeHandle nh;

    // Publisher for ROS node to provide waypoints pose
    ros::Publisher waypoints_pub = nh.advertise<geometry_msgs::PoseArray>("waypoints", 10);

    // Publisher for ROS node to provide object orientation pose
    ros::Publisher orientation_pub = nh.advertise<geometry_msgs::PoseStamped>("object_orientation", 10);

    // Subscribe to ROS node providing box dimensions and orientation
    ros::Subscriber sub = nh.subscribe<std_msgs::Float32MultiArray>("/pereption_topic", 1, perceptionDataCallback);

    // Wait for perception data and publish waypoints after receiving it
    ros::Rate rate(10); // Adjust the rate as needed
    while (ros::ok()) 
    {
        ros::spinOnce();
        publishWaypoints(waypoints_pub, orientation_pub);
        rate.sleep();
    }

    return 0;
}

