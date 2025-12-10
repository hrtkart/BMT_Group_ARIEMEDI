#include <iostream>
#include <chrono>
#include "NdiInterface.h"

int main() {
    // Create a NdiInterface instance
    NdiInterface tracker("169.254.8.69", "/home/jk/raad/NDI/sroms");
    
    // Setup the system
    tracker.setup();
    
    // Start tracking
    if (!tracker.startTracking()) {
        std::cout << "Could not start tracking" << std::endl;
        return 0;
    }
    // Start streaming
    tracker.startStreaming();

    // Stream data until error
    while (!tracker.isError()) {
        // Read stream data
        tracker.readStream();
        
        // Process and display
        tracker.processStreamData();
        
        // Your custom processing can go here
        // For example, accessing the pose data:
        const auto& poseData = tracker.getPoseData();
        // Iterate through the pose data
        for (const auto& pose : poseData) {
            std::cout << "Tool ID: " << pose.id << std::endl;
            
            if (!pose.is_missing) {
                // Access position data
                std::cout << "Position: " 
                        << *pose.tx << ", " 
                        << *pose.ty << ", " 
                        << *pose.tz << std::endl;
                        
                // Access orientation (quaternion)
                std::cout << "Orientation (w x y z): " 
                        << *pose.q0 << ", " 
                        << *pose.qx << ", " 
                        << *pose.qy << ", " 
                        << *pose.qz << std::endl;
                        
                // Access error value
                std::cout << "Error: " << *pose.error << std::endl;
            } else {
                std::cout << "Tool is not currently visible" << std::endl;
            }
            
            std::cout << "Frame: " << pose.frame << std::endl << std::endl;
        }
        
    }
    
    // Stop streaming
    tracker.stopStreaming();
    
    // Stop tracking
    tracker.stopTracking();
    
    return 0;
}