#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

// Define Point struct for positions
struct Point 
{
    
    double x, y, z;

};

// Function to calculate distance between two points (only Y coordinate)
double distanceY(const Point& p1, const Point& p2) 
{
    
    return std::abs(p2.y - p1.y);

}

// Function to generate waypoints based on initial and final points
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

// Function to write waypoints to a file for plotting
void writeWaypointsToFile(const std::vector<Point>& waypoints) 
{
    
    std::ofstream file("waypoints.txt");
    if (file.is_open()) 
    {
        
        for (const auto& waypoint : waypoints) 
        {
            
            file << waypoint.x << " " << waypoint.y << " " << waypoint.z << std::endl;

        }
        file.close();

    } 
    else 
    {
        
        std::cerr << "Unable to open file for writing!" << std::endl;

    }

}

int main() {
    // Initial position
    Point initial = {1.0, 2.0, 3.0};
    
    // Loop to simulate real-time input
    for (int i = 0; i < 3; ++i) 
    {
        
        std::cout << "\nRolling input data " << i + 1 << ":" << std::endl;
        
        // Simulating rolling input by reading positions from the user
        Point final;
        std::cout << "Enter final X coordinate: ";
        std::cin >> final.x;
        std::cout << "Enter final Y coordinate: ";
        std::cin >> final.y;
        std::cout << "Enter final Z coordinate: ";
        std::cin >> final.z;
        
        // Generate waypoints
        int numWaypoints = 5;
        std::vector<Point> waypoints = generateWaypoints(initial, final, numWaypoints);
        
        // Write waypoints to file for plotting
        writeWaypointsToFile(waypoints);
        
        // Set initial for next iteration
        initial = final;
        
    }

    return 0;
}

// g++ -o genRealWay genRealWay.cpp
// python3 plot_RealWay.py