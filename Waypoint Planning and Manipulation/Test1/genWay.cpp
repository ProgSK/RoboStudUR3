#include <iostream>
#include <vector>
#include <cmath>

struct Point 
{
    
    double x, y, z;

};

struct Pose 
{
    
    double x, y, z;
    double roll, pitch, yaw; // Orientation (in radians)

};

double distanceY(const Point& p1, const Point& p2) 
{
    
    return std::abs(p2.y - p1.y);

}

std::vector<Point> generateWaypoints(const Point& initial, const Point& final, int numWaypoints) 
{
    
    std::vector<Point> waypoints;
    
    // Calculate step sizes for each dimension
    double stepX = (final.x - initial.x) / (numWaypoints + 1);
    double stepY = (final.y - initial.y) / (numWaypoints + 1);
    double stepZ = (final.z - initial.z) / (numWaypoints + 1);
    
    // Generate equidistant waypoints
    for (int i = 1; i <= numWaypoints; ++i) 
    {
        
        Point waypoint;
        waypoint.x = initial.x + i * stepX;
        waypoint.y = initial.y + i * stepY;
        waypoint.z = initial.z + i * stepZ;
        waypoints.push_back(waypoint);

    }
    
    return waypoints;

}

std::vector<Pose> generatePoses(const std::vector<Point>& waypoints, double roll, double pitch, double yaw) 
{
    
    std::vector<Pose> poses;
    
    // Generate poses
    for (const auto& waypoint : waypoints) 
    {
        
        Pose pose;
        pose.x = waypoint.x;
        pose.y = waypoint.y;
        pose.z = waypoint.z;
        pose.roll = roll;  
        pose.pitch = pitch;
        pose.yaw = yaw;
        poses.push_back(pose);

    }

    return poses;

}

int main() 
{
    
    // // User input for initial and final positions
    // Point initial, final;
    // std::cout << "Enter initial X coordinate: ";
    // std::cin >> initial.x;
    // std::cout << "Enter initial Y coordinate: ";
    // std::cin >> initial.y;
    // std::cout << "Enter initial Z coordinate: ";
    // std::cin >> initial.z;
    
    // std::cout << "Enter final X coordinate: ";
    // std::cin >> final.x;
    // std::cout << "Enter final Y coordinate: ";
    // std::cin >> final.y;
    // std::cout << "Enter final Z coordinate: ";
    // std::cin >> final.z;
    
    // Mock dataset of initial and final position co-ordinates for waypoints
    Point initial = {2.0, 1.0, 3.0}; // X is the only variable that will change with linear box motion
    Point final = {50.0, 1.0, 3.0}; // X is the only variable that will change with linear box motion
    
    // Generate waypoints
    int numWaypoints = 12;
    std::vector<Point> waypoints = generateWaypoints(initial, final, numWaypoints);
    
    // Print generated waypoints
    std::cout << std::endl << "Generated Waypoints:" << std::endl << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        
        std::cout << "Waypoint " << i+1 << ": (" << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].z << ")" << std::endl;

    }
    
    // Calculate and print distances
    std::cout << std::endl << std::endl << "Distances:" << std::endl << std::endl;
    std::cout << "Initial to Waypoint 1: " << distanceY(initial, waypoints[0]) << std::endl;
    for (size_t i = 0; i < waypoints.size() - 1; ++i) 
    {
        
        std::cout << "Waypoint " << i+1 << " to Waypoint " << i+2 << ": " << distanceY(waypoints[i], waypoints[i+1]) << std::endl;
        
    }
    std::cout << "Waypoint " << waypoints.size() << " to Final: " << distanceY(waypoints.back(), final) << std::endl << std::endl;

    // Mock dataset of roll, pitch, yaw values for poses
    double roll = 5.0;
    double pitch = 5.0;
    double yaw = 18.0; // Yaw is the only variable that will change with box rotation on track
    
    // Generate poses
    std::vector<Pose> poses = generatePoses(waypoints, roll, pitch, yaw);
    
    // Print generated poses
    std::cout << std::endl << "Generated Poses:" << std::endl << std::endl;
    for (size_t i = 0; i < poses.size(); ++i) 
    {
        
        std::cout << "Pose " << i+1 << ": (" << poses[i].x << ", " << poses[i].y << ", " << poses[i].z << "), Roll: " << poses[i].roll << ", Pitch: " << poses[i].pitch << ", Yaw: " << poses[i].yaw << std::endl;
        
    }
    std::cout << std::endl;

    return 0;

}
