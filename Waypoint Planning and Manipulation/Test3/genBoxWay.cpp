#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>

struct Point 
{
    
    double x, y, z;

};

// Function to read waypoints from file
std::vector<Point> readWaypointsFromFile(const std::string& filename) 
{
    
    std::vector<Point> waypoints;
    std::ifstream file(filename);
    if (file.is_open()) 
    {
        
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Point waypoint;
            if (!(iss >> waypoint.x >> waypoint.y >> waypoint.z)) 
            { 
                
                break;

            }
            waypoints.push_back(waypoint);

        }
        file.close();

    } 
    else 
    {
        
        std::cerr << "Unable to open file " << filename << " for reading!" << std::endl;

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
    // Read waypoints from file
    std::vector<Point> waypoints = readWaypointsFromFile("waypoints.txt");
    if (waypoints.empty()) 
    {
        
        std::cerr << "No waypoints found in file!" << std::endl;
        return 1;

    }

    // Print the waypoints
    std::cout << "Waypoints:" << std::endl;
    for (const auto& point : waypoints) 
    {
        
        std::cout << point.x << " " << point.y << " " << point.z << std::endl;
        
    }

    // Write waypoints to file for plotting
    writeWaypointsToFile(waypoints);

    return 0;
}
