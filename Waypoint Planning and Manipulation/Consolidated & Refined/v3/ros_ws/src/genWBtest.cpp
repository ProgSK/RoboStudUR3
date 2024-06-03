// #include <iostream>
// #include <vector>
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <geometry_msgs/Point.h>

// // Define Point struct for positions
// struct Point 
// {
//     double x, y, z;
// };

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

// int main(int argc, char** argv) 
// {
//     // Static Y coordinate
//     double coodY = 0.3348766372496067;

//     // End-effector offset from box
//     double offsetZ = (0.7609185611395226 + 0.03);
    
//     // Initialize ROS node
//     ros::init(argc, argv, "waypoint_generator");
//     ros::NodeHandle nh;

//     // Publisher for ROS node to provide end-effector poses
//     ros::Publisher end_effector_pub = nh.advertise<geometry_msgs::PoseStamped>("waypoint_topic", 10);

//     // Publisher for waypoint markers
//     ros::Publisher waypoint_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_markers", 10);

//     // Publisher for box markers
//     ros::Publisher box_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("box_markers", 10);

//     // Define initial and final waypoints
//     Point initialWP = {0.01813680100817779, coodY, offsetZ};  // Modify with actual initial point as necessary
//     Point finalWP = {0.2608303568139733, coodY, offsetZ};    // Modify with actual final point as necessary

//     // Ensure node is ready and connections are established
//     ros::Duration(1.0).sleep();

//     // Publish initial waypoint
//     std::vector<Point> initialWaypoint = {initialWP};
//     publishWaypoints(initialWaypoint, end_effector_pub, "Initial Waypoint");

//     // Generate waypoints
//     int numWaypoints = 5;
//     std::vector<Point> waypoints = generateWaypoints(initialWP, finalWP, numWaypoints);

//     // Output initial and intermediate waypoints with orientation
//     for (int i = 0; i < waypoints.size(); ++i) 
//     {
//         std::cout << "Waypoint " << i + 1 << ": (" << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].z << "), Orientation: (1, 0, 0, 0)" << std::endl;
//     }

//     // Publish main waypoints
//     publishWaypoints(waypoints, end_effector_pub);

//     // Publish final waypoint
//     std::vector<Point> finalWaypoint = {finalWP};
//     publishWaypoints(finalWaypoint, end_effector_pub, "Final Waypoint");

//     // Add the waypoint that goes down by 0.2m at the final position
//     Point pickUpWP = {finalWP.x, finalWP.y, finalWP.z - 0.03};

//     // Add the waypoint that goes back up to the final position
//     Point returnWP = {finalWP.x, finalWP.y, finalWP.z};

//     // Define the drop-off point (hardcoded position)
//     Point dropOffWP = {0.3781727542368665, -0.268916276305741, 0.7676042784520107};

//     // Publish additional waypoints (pickup, return, and drop-off)
//     std::vector<Point> additionalWaypoints = {pickUpWP, returnWP, dropOffWP};
//     std::vector<std::string> messages = {"Pick-up Waypoint", "Return to Waypoint", "Drop-off Waypoint"};

//     for (size_t i = 0; i < additionalWaypoints.size(); ++i) 
//     {
//         publishWaypoints({additionalWaypoints[i]}, end_effector_pub, messages[i]);
//     }

//     // Publish waypoints as visualization markers including the initial, pickup and dropoff waypoints 
//     std::vector<Point> allWaypoints = initialWaypoint;
//     allWaypoints.insert(allWaypoints.end(), waypoints.begin(), waypoints.end());
//     allWaypoints.push_back(pickUpWP);
//     allWaypoints.push_back(returnWP);
//     allWaypoints.push_back(dropOffWP);
//     publishWaypointsMarkers(allWaypoints, waypoint_marker_pub);

//     // Publish box markers underneath waypoints
//     publishBoxMarkers(waypoints, box_marker_pub);

//     // Spin to keep the node running
//     ros::spin();

//     return 0;
// }


#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

// Define Point struct for positions
struct Point 
{
    double x, y, z;
};

// Define Orientation struct for quaternion orientation
struct Orientation 
{
    double x, y, z, w;
};

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

// Function to publish waypoints as end-effector poses and print the corresponding message
void publishWaypoints(const std::vector<Point>& waypoints, ros::Publisher& publisher, const std::string& message = "", const Orientation* orientation = nullptr) 
{
    for (const auto& waypoint : waypoints) 
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map"; // Assuming waypoints are in map frame
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = waypoint.x;
        pose_msg.pose.position.y = waypoint.y;
        pose_msg.pose.position.z = waypoint.z;
        
        // Set orientation
        if (orientation) {
            pose_msg.pose.orientation.x = orientation->x;
            pose_msg.pose.orientation.y = orientation->y;
            pose_msg.pose.orientation.z = orientation->z;
            pose_msg.pose.orientation.w = orientation->w;
        } else {
            pose_msg.pose.orientation.x = 1.0; // Default orientation
            pose_msg.pose.orientation.y = 0.0;
            pose_msg.pose.orientation.z = 0.0;
            pose_msg.pose.orientation.w = 0.0;
        }

        publisher.publish(pose_msg);
        if (!message.empty())
        {
            std::cout << message << ": (" << waypoint.x << ", " << waypoint.y << ", " << waypoint.z << "), Orientation: ("
                      << pose_msg.pose.orientation.x << ", " << pose_msg.pose.orientation.y << ", " 
                      << pose_msg.pose.orientation.z << ", " << pose_msg.pose.orientation.w << ")" << std::endl;
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

int main(int argc, char** argv) 
{
    // Static Y coordinate
    double coodY = 0.3408323009617693;

    // End-effector offset from box
    double offsetZ = (0.770103836325007 + 0.03);
    
    // Initialize ROS node
    ros::init(argc, argv, "waypoint_generator");
    ros::NodeHandle nh;

    // Publisher for ROS node to provide end-effector poses
    ros::Publisher end_effector_pub = nh.advertise<geometry_msgs::PoseStamped>("waypoint_topic", 10);

    // Publisher for waypoint markers
    ros::Publisher waypoint_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_markers", 10);

    // Publisher for box markers
    ros::Publisher box_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("box_markers", 10);

    // Define initial and final waypoints
    Point initialWP = {0.01813680100817779, coodY, offsetZ};  // Modify with actual initial point as necessary
    Point finalWP = {0.2595913582511396, coodY, offsetZ};    // Modify with actual final point as necessary

    // Ensure node is ready and connections are established
    ros::Duration(1.0).sleep();

    // Publish initial waypoint
    std::vector<Point> initialWaypoint = {initialWP};
    publishWaypoints(initialWaypoint, end_effector_pub, "Initial Waypoint");

    // Generate waypoints
    int numWaypoints = 5;
    std::vector<Point> waypoints = generateWaypoints(initialWP, finalWP, numWaypoints);

    // Output initial and intermediate waypoints with orientation
    for (int i = 0; i < waypoints.size(); ++i) 
    {
        std::cout << "Waypoint " << i + 1 << ": (" << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].z << "), Orientation: (1, 0, 0, 0)" << std::endl;
    }

    // Publish main waypoints
    publishWaypoints(waypoints, end_effector_pub);

    // Publish final waypoint
    std::vector<Point> finalWaypoint = {finalWP};
    publishWaypoints(finalWaypoint, end_effector_pub, "Final Waypoint");

    // Add the waypoint that goes down by 0.2m at the final position
    Point pickUpWP = {finalWP.x, finalWP.y, finalWP.z - 0.03};

    // Add the waypoint that goes back up to the final position
    Point returnWP = {finalWP.x, finalWP.y, finalWP.z};

    // Define the drop-off point (hardcoded position)
    Point dropOffWP = {0.3781727542368665, -0.268916276305741, 0.7676042784520107};
    Orientation dropOffOrientation = {-0.4259426688592926, -0.904699954572914, 0.0023730564180625448, 0.009230581998290781};

    // Publish additional waypoints (pickup, return, and drop-off)
    std::vector<Point> additionalWaypoints = {pickUpWP, returnWP, dropOffWP};
    std::vector<std::string> messages = {"Pick-up Waypoint", "Return to Waypoint", "Drop-off Waypoint"};

    for (size_t i = 0; i < additionalWaypoints.size(); ++i) 
    {
        if (i == 2) { // For drop-off waypoint, use the specific orientation
            publishWaypoints({additionalWaypoints[i]}, end_effector_pub, messages[i], &dropOffOrientation);
        } else {
            publishWaypoints({additionalWaypoints[i]}, end_effector_pub, messages[i]);
        }
    }

    // Publish waypoints as visualization markers including the initial, pickup and dropoff waypoints 
    std::vector<Point> allWaypoints = initialWaypoint;
    allWaypoints.insert(allWaypoints.end(), waypoints.begin(), waypoints.end());
    allWaypoints.push_back(pickUpWP);
    allWaypoints.push_back(returnWP);
    allWaypoints.push_back(dropOffWP);
    publishWaypointsMarkers(allWaypoints, waypoint_marker_pub);

    // Publish box markers underneath waypoints
    publishBoxMarkers(waypoints, box_marker_pub);

    // Spin to keep the node running
    ros::spin();

    return 0;
}
