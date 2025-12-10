#include "NdiInterface.h"

NdiInterface::NdiInterface(const std::string &host, const std::string &toolDir)
    : capi()
    , cipher("")
    , hostname(host)
    , streamedReply("")
    , toolsDirectory(toolDir)
    , streamId("tcpStream1")
{}

NdiInterface::~NdiInterface()
{
    // No explicit disconnect needed
}

const std::vector<NdiInterface::PoseData> &NdiInterface::getPoseData() const
{
    return poseDataResults;
}

const std::string &NdiInterface::getStreamedReply() const
{
    return streamedReply;
}

bool NdiInterface::fileExists(const std::string &filename)
{
    return ACCESS(filename.c_str(), 0) == 0;
}

void NdiInterface::sleepSeconds(unsigned numSeconds)
{
    sleep(numSeconds);
}

void NdiInterface::onErrorPrintDebugMessage(std::string methodName, int errorCode)
{
    if (errorCode < 0) {
        std::cout << methodName << " failed: " << capi.errorToString(errorCode) << std::endl;
    }
}

std::string NdiInterface::getToolInfo(std::string toolHandle)
{
    PortHandleInfo info = capi.portHandleInfo(toolHandle);
    std::string outputString = info.getToolId();
    outputString.append(" s/n:").append(info.getSerialNumber());
    return outputString;
}

std::vector<NdiInterface::PoseData> NdiInterface::parsePosePacket(const std::string &packet)
{
    std::vector<PoseData> result;
    std::istringstream iss(packet);
    std::string line;

    // Read the first line (number of records)
    std::getline(iss, line);
    int num_records = std::stoi(line.substr(0, 2));
    line.erase(0, 2);

    // Process each record
    for (int i = 0; i < num_records; i++) {
        PoseData pose;

        // Extract ID (first 2 characters)
        pose.id = std::stoi(line.substr(0, 2));

        // Check if this is a "MISSING" entry
        if (line.find("MISSING") != std::string::npos) {
            pose.is_missing = true;

            // Extract frame from the MISSING line at the same position
            // Convert from hex to int
            std::string frameHex = line.substr(line.length() - 6, 6);
            pose.frame = std::stoi(frameHex, nullptr, 16);
        } else {
            pose.is_missing = false;

            // Parse the data fields
            pose.q0 = std::stoi(line.substr(2, 6)) / 10000.0;
            pose.qx = std::stoi(line.substr(8, 6)) / 10000.0;
            pose.qy = std::stoi(line.substr(14, 6)) / 10000.0;
            pose.qz = std::stoi(line.substr(20, 6)) / 10000.0;
            pose.tx = std::stoi(line.substr(26, 7)) / 100.0;
            pose.ty = std::stoi(line.substr(33, 7)) / 100.0;
            pose.tz = std::stoi(line.substr(40, 7)) / 100.0;
            pose.error = std::stoi(line.substr(47, 6)) / 10000.0;

            // Extract frame, convert from hex to int
            std::string frameHex = line.substr(63, 6);
            pose.frame = std::stoi(frameHex, nullptr, 16);
        }

        result.push_back(pose);
        if (!std::getline(iss, line)) {
            break; // End of input
        }
    }

    // Store the result in the class member for external access
    poseDataResults = result;

    return result;
}

void NdiInterface::printPoseData(const std::vector<PoseData> &poses)
{
    std::cout << "Parsed " << poses.size() << " pose records:" << std::endl;

    for (const auto &pose : poses) {
        std::cout << "ID: " << pose.id << std::endl;

        if (pose.is_missing) {
            std::cout << "  Status: MISSING" << std::endl;
        } else {
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "  q0: " << *pose.q0 << std::endl;
            std::cout << "  qx: " << *pose.qx << std::endl;
            std::cout << "  qy: " << *pose.qy << std::endl;
            std::cout << "  qz: " << *pose.qz << std::endl;
            std::cout << "  tx: " << *pose.tx << std::endl;
            std::cout << "  ty: " << *pose.ty << std::endl;
            std::cout << "  tz: " << *pose.tz << std::endl;
            std::cout << "  error: " << *pose.error << std::endl;
        }

        std::cout << "  frame: " << pose.frame << std::endl;
        std::cout << std::endl;
    }
}

void NdiInterface::startStreaming()
{
    capi.startStreaming("TX 0801", streamId, Protocol::TCP, cipher);
}

void NdiInterface::stopStreaming()
{
    capi.stopStreaming(streamId, Protocol::TCP);
}

std::string NdiInterface::readStream()
{
    streamedReply = capi.readStream(streamId);
    return streamedReply;
}

void NdiInterface::processStreamData()
{
    std::vector<PoseData> poses = parsePosePacket(streamedReply);
    // printPoseData(poses);
}

void NdiInterface::calculateFrameRate(int frameCount,
                                      std::chrono::high_resolution_clock::time_point &startTime,
                                      double &frameRate)
{
    const int RATE_UPDATE_INTERVAL = 100; // Update rate calculation every 100 frames

    // Calculate frequency every RATE_UPDATE_INTERVAL frames
    if (frameCount % RATE_UPDATE_INTERVAL == 0) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedTime
            = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

        // Calculate frames per second
        frameRate = (RATE_UPDATE_INTERVAL * 1000.0) / elapsedTime;

        // Reset timer for next calculation
        startTime = currentTime;
        std::cout << std::fixed << std::setprecision(2) << frameRate << "Hz " << std::endl;
    }
}

void NdiInterface::initializeAndEnableTools()
{
    std::cout << std::endl << "Initializing and enabling tools..." << std::endl;

    // Initialize and enable tools
    std::vector<PortHandleInfo> portHandles = capi.portHandleSearchRequest(
        PortHandleSearchRequestOption::NotInit);
    for (std::size_t i = 0; i < portHandles.size(); i++) {
        onErrorPrintDebugMessage("capi.portHandleInitialize()",
                                 capi.portHandleInitialize(portHandles[i].getPortHandle()));
        onErrorPrintDebugMessage("capi.portHandleEnable()",
                                 capi.portHandleEnable(portHandles[i].getPortHandle()));
    }

    // Print all enabled tools
    portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
    for (std::size_t i = 0; i < portHandles.size(); i++) {
        std::cout << portHandles[i].toString() << std::endl;
    }

    // Lookup and store the serial number for each enabled tool
    for (std::size_t i = 0; i < portHandles.size(); i++) {
        enabledTools.push_back(ToolData());
        enabledTools.back().transform.toolHandle = (uint16_t) capi.stringToInt(
            portHandles[i].getPortHandle());
        enabledTools.back().toolInfo = getToolInfo(portHandles[i].getPortHandle());
    }

    // Initialize poseDataResults with one entry for each enabled tool
    poseDataResults.clear();
    for (std::size_t i = 0; i < enabledTools.size(); i++) {
        PoseData emptyPose;
        emptyPose.id = i;
        emptyPose.is_missing = true;
        emptyPose.frame = 0;
        poseDataResults.push_back(emptyPose);
    }
}

void NdiInterface::setToolsDirectory(const std::string &directory)
{
    toolsDirectory = directory;
}

std::string NdiInterface::getToolsDirectory() const
{
    return toolsDirectory;
}

void NdiInterface::loadToolsFromDirectory()
{
    std::cout << "Loading Tool Definitions from directory: " << toolsDirectory << std::endl;

    if (!fileExists(toolsDirectory)) {
        std::cerr << "Tool directory does not exist: " << toolsDirectory << std::endl;
        return;
    }

    // Iterate through directory and find .rom files
    std::vector<std::filesystem::directory_entry> entries;

    // Collect all .rom files into the vector
    for (const auto &entry : std::filesystem::directory_iterator(toolsDirectory)) {
        if (entry.path().extension() == ".rom") {
            entries.push_back(entry);
        }
    }

    // Sort the entries by filename (assumes filenames are numbers)
    std::sort(entries.begin(), entries.end(), [](const auto &a, const auto &b) {
        // Convert filenames to integers for numerical sorting
        return std::stoi(a.path().stem().string()) < std::stoi(b.path().stem().string());
    });

    size_t toolCount = 0;
    for (const auto &entry : entries) {
        std::string toolPath = entry.path().string();
        std::cout << "Loading: " << toolPath << std::endl;
        loadTool(toolPath.c_str());
        toolCount++;
    }

    // Pre-allocate poseDataResults based on number of tools found
    poseDataResults.clear();
    for (size_t i = 0; i < toolCount; i++) {
        PoseData emptyPose;
        emptyPose.id = i;
        emptyPose.is_missing = true;
        emptyPose.frame = 0;
        poseDataResults.push_back(emptyPose);
    }
}

void NdiInterface::loadTool(const char *toolDefinitionFilePath)
{
    // Request a port handle to load a passive tool into
    int portHandle = capi.portHandleRequest();
    onErrorPrintDebugMessage("capi.portHandleRequest()", portHandle);

    // Load the .rom file using the previously obtained port handle
    capi.loadSromToPort(toolDefinitionFilePath, portHandle);
}

void NdiInterface::configureUserParameters()
{
    std::cout << capi.getUserParameter("Param.User.String0") << std::endl;
    onErrorPrintDebugMessage("capi.setUserParameter(Param.User.String0, customString)",
                             capi.setUserParameter("Param.User.String0", "customString"));
    std::cout << capi.getUserParameter("Param.User.String0") << std::endl;
    onErrorPrintDebugMessage("capi.setUserParameter(Param.User.String0, emptyString)",
                             capi.setUserParameter("Param.User.String0", ""));
}

void NdiInterface::simulateAlerts(uint32_t simulatedAlerts)
{
    // Simulate alerts if any were requested
    if (simulatedAlerts > 0x0000) {
        std::cout << std::endl << "Simulating system alerts..." << std::endl;
        std::stringstream stream;
        stream << simulatedAlerts;
        onErrorPrintDebugMessage("capi.setUserParameter(Param.Simulated Alerts, alerts)",
                                 capi.setUserParameter("Param.Simulated Alerts", stream.str()));
        std::cout << capi.getUserParameter("Param.Simulated Alerts") << std::endl;
    }
}

bool NdiInterface::connect()
{
    if (capi.connect(hostname, Protocol::TCP, cipher) != 0) {
        std::cout << "Connection Failed!" << std::endl;
        return false;
    }
    std::cout << "Connected!" << std::endl;

    // Wait a second - needed to support connecting to LEMO Vega
    sleepSeconds(1);

    // Initialize the system
    onErrorPrintDebugMessage("capi.initialize()", capi.initialize());

    return true;
}

bool NdiInterface::startTracking()
{
    std::cout << std::endl << "Entering tracking mode..." << std::endl;
    int result = capi.startTracking();
    onErrorPrintDebugMessage("capi.startTracking()", result);
    return result >= 0;
}

bool NdiInterface::stopTracking()
{
    std::cout << std::endl
              << "Leaving tracking mode and returning to configuration mode..." << std::endl;
    int result = capi.stopTracking();
    onErrorPrintDebugMessage("capi.stopTracking()", result);
    return result >= 0;
}

bool NdiInterface::isError()
{
    return streamedReply.rfind("ERROR", 0) == 0;
}

void NdiInterface::setup()
{
    if (!connect()) {
        return;
    }

    // Load tools from directory
    loadToolsFromDirectory();

    // Initialize and enable loaded tools
    initializeAndEnableTools();

    // Print an error if no tools were detected
    if (enabledTools.size() == 0) {
        std::cout << "No tools detected. Please check the tool directory: " << toolsDirectory
                  << std::endl;
    }
}