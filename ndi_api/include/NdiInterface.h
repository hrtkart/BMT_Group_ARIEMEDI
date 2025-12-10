#ifndef NDI_INTERFACE_H
#define NDI_INTERFACE_H

#define ACCESS access
#include <sys/ioctl.h>
#include <unistd.h> // for POSIX sleep(sec), and access()

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"
#include <algorithm>
class NdiInterface
{
public:
    struct PoseData
    {
        int id;
        bool is_missing;
        std::optional<double> q0, qx, qy, qz;
        std::optional<double> tx, ty, tz;
        std::optional<double> error;
        int frame;
    };

private:
    CombinedApi capi;
    std::string cipher;
    std::string hostname;
    std::string streamedReply;
    std::vector<ToolData> enabledTools;
    std::string toolsDirectory;
    std::vector<PoseData> poseDataResults;
    std::string streamId;

public:
    NdiInterface(const std::string &host = "169.254.8.69", const std::string &toolDir = ".");
    ~NdiInterface();

    // Getters
    const std::vector<PoseData> &getPoseData() const;
    const std::string &getStreamedReply() const;
    std::string getToolsDirectory() const;

    // Setters
    void setToolsDirectory(const std::string &directory);

    // Helper functions
    bool fileExists(const std::string &filename);
    void sleepSeconds(unsigned numSeconds);
    void onErrorPrintDebugMessage(std::string methodName, int errorCode);
    std::string getToolInfo(std::string toolHandle);

    // Pose data handling
    std::vector<PoseData> parsePosePacket(const std::string &packet);
    void printPoseData(const std::vector<PoseData> &poses);

    // Streaming functions
    void startStreaming();
    void stopStreaming();
    std::string readStream();
    void processStreamData();
    void calculateFrameRate(int frameCount,
                            std::chrono::high_resolution_clock::time_point &startTime,
                            double &frameRate);
    bool isError();

    // Tool management
    void initializeAndEnableTools();
    void loadToolsFromDirectory();
    void loadTool(const char *toolDefinitionFilePath);

    // System configuration
    void configureUserParameters();
    void simulateAlerts(uint32_t simulatedAlerts = 0x00000000);

    // Connection management
    bool connect();
    bool startTracking();
    bool stopTracking();

    // Setup function
    void setup();
};

#endif // NDI_INTERFACE_H