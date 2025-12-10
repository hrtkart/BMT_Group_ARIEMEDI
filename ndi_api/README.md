# API Reference

## Build
```
mkdir build
cd build
cmake ..
cmake --build .
```

## Class: NdiInterface

### Constructor
```
NdiInterface(const std::string& host = "169.254.8.69", const std::string& toolDir = ".");
```
- host: IP address of the NDI tracking system
- toolDir: Directory containing tool definition (.rom) files

### Key Methods
```
void setup();  // Connect to the NDI system and load tools

bool startTracking();  // Start the tracking system
bool stopTracking();   // Stop the tracking system

void startStreaming();  // Start streaming data
void stopStreaming();   // Stop streaming data
std::string readStream();  // Read a single frame of data

bool isError();  // Check if an error has occurred in the data stream

const std::vector<PoseData>& getPoseData() const;  // Get the latest pose data
```

### Struct: PoseData
This structure contains the tracking information for a single tool:
```
struct PoseData {
    int id;             // Tool ID
    bool is_missing;    // Whether the tool is currently visible
    
    // These values are only valid when is_missing is false
    std::optional<double> q0, qx, qy, qz;  // Quaternion orientation (w, x, y, z)
    std::optional<double> tx, ty, tz;      // Position in mm
    std::optional<double> error;           // Error value
    
    int frame;          // Frame number
};
```