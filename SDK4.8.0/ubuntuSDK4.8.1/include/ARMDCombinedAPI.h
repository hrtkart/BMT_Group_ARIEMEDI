#ifndef ARMDCOMBINEDAPI_H
#define ARMDCOMBINEDAPI_H

#ifdef ARMDCOMBINEDAPI_EXPORTS
#define ARMDCOMBINED_API __declspec(dllexport)
#else
#define ARMDCOMBINED_API __declspec(dllimport)
#endif

#include "Tracking.h"

#include <map>
#include <numeric>
#include <algorithm>
#include <cctype>
#include <regex>
#include <chrono>
#include <iomanip>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>
#include <float.h>
#include <linux/sockios.h>
#include <linux/ethtool.h>

namespace DeviceType
{
	enum value
	{
		None = 0x0000,
		SE = 0x0001,//tracking(120HZ)+imaging(1280*800)
		LITE = 0x0002,//tracking(120HZ)+imaging(1280*800)
		MAX = 0x0003,//tracking(120HZ)+imaging(1280*800)
		//0x0004,
		//0x0005,
		MAXV = 0x0006,//tracking(120HZ)+imaging(1280*800)+monitoring(laser)
		MAXS = 0x0007,//tracking(400HZ)+imaging(1280*800)+monitoring(laser)
		//0x0008,
		SED = 0x0009,//tracking(120HZ)+imaging(1920*1200)+monitoring(laser)+reconstruction
		PRO = 0x000A,//tracking(120HZ)+imaging(1920*1200)+monitoring(laser)+reconstruction
		SEDP = 0x000B,//tracking(400HZ)+imaging(1920*1200)+monitoring(laser)+reconstruction
		ULTR = 0x000C,//tracking(400HZ)+imaging(1920*1200)+monitoring(laser)+reconstruction
	};
}

namespace GenerationStatus
{
	enum value
	{
		OK = 0x0000,
		MinMatchedNumLess = 0x0001,
		MinMatchedNumMore = 0x0002,
		PlaneNumLess = 0x0003,
		PlaneNumMore = 0x0004,
		PlaneNumNotMatching = 0x0005,
		MarkerNumLess = 0x0006,
		MarkerNumMore = 0x0007,
		MarkerNumLimitExceed = 0x0008,
		DistanceTooSmall = 0x0009,
		NameError = 0x000A,
		PathNotExist = 0x000B,
		TypeError = 0x000C,
		StructureTypeError = 0x000D,
	};

	//return the std::string representation of the AROM generation status
	std::string toString(uint16_t generationCode);
}

namespace TransmissionStatus
{
	enum value
	{
		NoneData = 0x0000,
		TrackingData = 0x0001,
		ImagingData = 0x0002,
		AllData = 0x0003,
		Suspend = 0x0004,
		ReconstructionPrepare = 0x0005,
		ReconstructionReady = 0x0006,
		LMonitor = 0x0007,
		HMonitor = 0x0008,
	};
}

namespace TransmissionType
{
	enum value
	{
		None = 0x0000,
		Passive = 0x0001,
		Active = 0x0002,
		Mixed = 0x0003,
		Alternate = 0x0004,
	};
}

namespace ConnectionStatus
{
	enum value
	{
		Connected = 0x0000,
		DisConnected = 0x0001,
		DisConnectedFailed = 0x0002,
		PortConflicts = 0x0003,
		ResloveError = 0x0004,
		InitialUDPError = 0x0005,
		Interruption = 0x0006,
		InvalidAddr = 0x0007,
		DeviceInexist = 0x0008,
		TimeOut = 0x0009,
		NotReachable = 0x000A,
		UnknownError = 0x000B,
	};

	//return the std::string representation of the connection status
	std::string toString(uint16_t conditionCode);
}

namespace DeviceAlert
{
	enum value
	{
		Normal = 0x0000,
		Vibration = 0x0001,
		TempTooHigh = 0x0002,
		TempTooLow = 0x0003,
	};

	//return the std::string representation of the device alert
	std::string toString(uint16_t alertCode);
}

namespace CalibrationAlert
{
	enum value
	{
		Normal = 0x0000,
		AbnormalData = 0x0001,
		InadequateData = 0x0002,
		ComputationError = 0x0003,
	};

	//return the std::string representation of the calibration alert
	std::string toString(uint16_t alertCode);
}

struct DeviceInfo
{
	std::string hostname = "";//RT device hostname
	std::string IP = "";//RT device IP address
	std::string MAC = "";//RT device MAC address
	int deviceType = DeviceType::None;//RT device type
	int firmwareVersion[3] = { 0, 0, 0 };//RT device firmware version
};

struct ToolCalibrationData
{
	std::string name = "";//tool name
	bool markerType = false;//passive(false) or active(true) tool
	int minNumMarker = 3;//minmum number of markers to be matched
	int planeNum = 1;//plane number
	std::vector<MarkerPlane> planes;//maker information and direction per plane
	std::vector<MarkerPosition> points;//virtual point position
	double dir[3] = { 0.0, 0.0, 0.0 };//virtual direction
	double pin[3] = { 0.0, 0.0, 0.0 };//tip position
	double calbError = -1;//calibration error
	double maxFRE = 1;//maximum matching FRE
	double maxAngle = 60;//maximum angle for matching
	int structureType = 0;//tool structure
	int algorithmType = 1;//algorithm setting
};

struct NetAdaptorInfo
{
	std::string name = "";//net adaptor name
	std::string IP = "";//net adaptor IP addrese
	std::string MAC = "";//net adaptor MAC address
	std::string connectionType = "";//net adaptor connection type
	std::string mask = "";//net adaptor mask
	int speed = 0;//net speed
	bool linked = false;//link status
};

class ARMDCombinedAPI
{
public:
	ARMDCombinedAPI();
	virtual ~ARMDCombinedAPI();

	/**
	* @brief: connect RT device
	* @param1: hostnameor/IP address of the device
	* @param2: auto or fixed exposure
	* @param3: show or hide the UDP error
	* @return: connection result
	*/
	int connect(std::string hostname, bool tripleBeep = true, bool autoExposure = true, bool errorPrint = false);

	/**
	* @brief: disconnect RT device
	*/
	void disconnect();

	/**
	* @brief: get connected device information
	* @return: a struct of device information, including hostname, IP address, MAC address
	*/
	DeviceInfo getConnectedDeviceInfo();

	/**
	* @brief: print API and firmware version
	*/
	std::vector<int> printAPIandFirmwareVersion();

	/**
	* @brief: set tracking data transmission type
	* @param: transmission type: 1-only passive markers(default), 2-only active markers, 3-mixed transamission, 4-alternate transmission
	*/
	void setTrackingDataTransmissionType(int type);

	/**
	* @brief: set transformation matrix for original tracking data 
	* @param: transformation matrix
	* @return: true-success, false-failed
	* @warning: matrix will be automaticlly orthogonalized 
	*/
	bool setTrackingDataTransformation(double matrix[4][4]);

	/**
	* @brief: set filter method for tracking data
	* @param1: filter method ("mean", "kalman")
	* @param2: data number for mean method
	*/
	void setTrackingDataFilterMethod(std::string method, int number = 10);

	/**
	* @brief: load ARMD passive tool definition
	* @param1: path of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadPassiveToolAROM(std::string path, std::string parm = "cover");

	/**
	* @brief: load ARMD passive tool definition
	* @param1: vector of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadPassiveToolAROM(std::vector<std::string> paths, std::string parm = "cover");

	/**
	* @brief: load ARMD wireless tool definition
	* @param1: path of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadActiveWirelessToolAROM(std::string path, std::string parm = "cover");

	/**
	* @brief: load ARMD wireless tool definition
	* @param1: vector of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadActiveWirelessToolAROM(std::vector<std::string> paths, std::string parm = "cover");

	/**
	* @brief: load ARMD active tool definition
	* @param1: path of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadActiveToolAROM(std::string path, std::string parm = "cover");

	/**
	* @brief: load ARMD active tool definition
	* @param1: vector of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadActiveToolAROM(std::vector<std::string> paths, std::string parm = "cover");

	/**
	* @brief: load non-ARMD tool definition (will be forbidden in future)
	* @param1: path of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	* @warning: this function is risky and not official file parsing of the corresponding type
	*/
	void loadPassiveToolNAROM(std::string path, std::string parm = "cover");

	/**
	* @brief: load non-ARMD tool definition (will be forbidden in future)
	* @param1: vector of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	* @warning: this function is risky and not official file parsing of the corresponding type
	*/
	void loadPassiveToolNAROM(std::vector<std::string> paths, std::string parm = "cover");

	/**
	* @brief: enable or unenable tracking of the specified tool 
	* @param: map of tool name and enable/unenable
	*/
	void setToolTrackingEnable(std::map<std::string, bool> setting);

	/**
	* @brief: set luminance of the IF images from the cameras
	* @param1: luminance
	* @param2: choose the images from left(1)/right(2)/both(0, default) cameras
	*/
	void setIFLuminance(double l, int side = 0);

	/**
	* @brief: set contrast of the IF images from the cameras
	* @param1: contrast
	* @param2: choose the images from left(1)/right(2)/both(0, default) cameras
	*/
	void setIFContrast(double c, int side = 0);

	/**
	* @brief: draw area on the IF images from the cameras
	* @param1: width (pixels) of area
	* @param2: height (pixels)of area
	* @param3: start position of area in the x-direction of the IF image from left camera
	* @param4: start position of area in the y-direction of the IF image from left camera
	* @param5: start position of area in the x-direction of the IF image from right camera
	* @param6: start position of area in the y-direction of the IF image from right camera
	*/
	void setAreaDisplay(short width, short height, short leftX, short leftY, short rightX, short rightY);

	/**
	* @brief: hide the marked area on the IF images from the cameras
	*/
	void setAreaHidden();

	/**
	* @brief: set the resolution of visible light image from the monitor
	* @param: resolution level: 0-low(1280*800), 1-high(1920*1200, default)
	* @warning: this function only works for the devices of RT-PRO and RT-MAXV
	*/
	void setVLImageResolution(bool level);

	/**
	* @brief: get the size of infrared images from the cameras
	* @return: vector of image width and height
	*/
	std::vector<int> getIFImageSize();

	/**
	* @brief: get the size of visible light image from the monitor
	* @return: vector of image width and height
	* @warning: this function only works for the devices of RT-PRO and RT-MAXV
	*/
	std::vector<int> getVLImageSize();

	/**
	* @brief: get tool information
	* @return: calibration information of all tools
	*/
	std::vector<ToolCalibrationData> getToolStorage() { return this->trackTools; };

	/**
	* @brief: clear all tool information
	*/
	void clearToolStorage();

	/**
	* @brief: get number of tools
	* @return: number of tools
	*/
	int getTrackToolsNum() { return this->trackTools.size(); };

	/**
	* @brief: start tracking
	*/
	void startTracking();

	/**
	* @brief: stop tracking
	*/
	void stopTracking();

	/**
	* @brief: start monitor
	* @param: resolution level: 0-low(1280*720), 1-high(1920*1200, default)
	*/
	void startVideoMonitor(bool r = true);

	/**
	* @brief: stop monitor
	*/
	void stopVideoMonitor();

	/**
	* @brief: start imaging
	*/
	void startImaging();

	/**
	* @brief: stop imaging
	*/
	void stopImaging();

	/**
	* @brief: start reconstruction
	* @param1: width (pixels) of reconstruction area
	* @param2: height (pixels)of reconstruction area
	* @param3: start position of reconstruction area in the x-direction of the image from left camera
	* @param4: start position of reconstruction area in the y-direction of the image from left camera
	* @param5: start position of reconstruction area in the x-direction of the image from right camera  
	* @param6: start position of reconstruction area in the y-direction of the image from right camera  
	* @warning: this function only works for the device of RT-PRO
	*/
	void reconstructPointCloud(short width, short height, short leftX, short leftY, short rightX, short rightY);

	/**
	* @brief: evaluate the reconstruction result
	* @param1: the reconstructed point cloud stage
	* @return: true-good, false-bad
	* @warning: this function only works for the device of RT-PRO
	*/
	bool evaluateReconstruction(int stage);

	/**
	* @brief: set material of object to be reconstructed
	* @param1: 0-plaster(default), 1-silicone
	* @return: true-success, false-failed
	* @warning: this function only works for the device of RT-PRO
	*/
	void setMaterial(int material);

	/**
	* @brief: update tracking and imaging data
	*/
	int trackingUpdate();

	/**
	* @brief: update video monitor data
	* @warning: this function only works for the device of RT-PRO
	*/
	void monitoringUpdate();

	/**
	* @brief: get all markers with the type
	* @param: marker type: 0-all(default), 1-passive, 2-active, 3-real, 4-phantom
	* @return: positions of all markers with the type
	*/
	std::vector<MarkerPosition> getAllMarkers(int type = 0);

	/**
	* @brief: get current tracking data
	* @param: all detected markers
	* @return: tracking data of tool
	*/
	std::vector<ToolTrackingData> getTrackingData(std::vector<MarkerPosition> cordi);

	/**
	* @brief: get enable or unenable tracking information of all tools
	* @return: map of tool name and enable/unenable tracking information
	*/
	std::map<std::string, bool> getToolTrackingEnableInfo() { return this->trackingEnable; };

	/**
	* @brief: get stray markers
	* @return: positions of all stray markers
	*/
	std::vector<MarkerPosition> getUnMatchedMarkers() { return this->unMatchedMarkers; };

	/**
	* @brief: set imaging mode for alternate transmission
	* @param: 0-passive, 1-active
	* @warning: this function only works for the alternate transmission, if not set, the passive imaging data is default
	*/
	void setImagingMode(int mode);

	/**
	* @brief: get current imaging data
	* @return: image from left camera
	*/
	char* getLeftImagingData();

	/**
	* @brief: get current imaging data
	* @return: image from right camera
	*/
	char* getRightImagingData();

	/**
	* @brief: get current video data
	* @return: image from video monitor
	*/
	unsigned char* getVideoMonitorData();

	/**
	* @brief: get reconstruction data
	* @return: reconstruction data
	* @warning: this function only works for the device of RT-PRO
	*/
	std::vector<std::vector<char>> getReconstructionData();

	/**
	* @brief: get system time
	* @return: system time
	*/
	std::string getSystemTime();

	/**
	* @brief: get parameters for reconstruction
	* @return: parameters
	* @warning: this function only works for the device of RT-PRO
	*/
	std::vector<double> getReconstructParameters() { return this->RCparameters; };

	/**
	* @brief: get gravity vector
	* @return: gravity vector
	*/
	std::vector<double> getGravityVector() { return this->gravity; };

	/**
	* @brief: get system status
	* @return: connection status
	*/
	uint16_t getConnectionStatus() { return this->connectionStatus; };

	/**
	* @brief: get system status
	* @return: transmission status
	*/
	uint16_t getTransmissionStatus() { return this->transmissionStatus; };

	/**
	 * @brief: get reconstruction status
	 * @return: reconstruction status
	 */
	uint16_t getReconstructionStatus() { return this->reconstructionStatus; };

	/**
	* @brief: get system status
	* @return: system alert
	*/
	uint16_t getSystemAlert() { return this->systemAlert; };

	/**
	* @brief: create an arom file
	* @param1: file directory to be saved
	* @param2: tool calibration struct
	* @return: generation status
	*/
	uint16_t generateAROM(std::string directory, ToolCalibrationData tool, double radius = 5);

	/**
	* @brief: convert a non-arom calibration file to an standard arom file
	* @param1: file directory to be saved
	* @param2: path of non-arom calibration file
	* @return: generation status
	*/
	uint16_t convert2AROM(std::string directoryOut, std::string directoryIn);

	/**
	* @brief: pivot calibration for a tool tip
	* @param1: rotation matrix (3n*3) during pivot calibration
	* @param2: translation vection (3n*1) during pivot calibration
	* @param3: (output) calibrated tip (3*1)
	* @param4: (output) calibration error
	* @return: pivot calibration alert
	*/
	uint16_t pivotTipCalibration(std::vector<std::vector<double>> rot, std::vector<std::vector<double>> tran, std::vector<double>& tip, double& error);
	
	/**
	* @brief: pivot calibration for a tool tip
	* @param1: tool calibration data
	* @param2: tracking data during pivot calibration
	* @param3: (output) calibrated tip in the coordinate of the tool
	* @param4: (output) tip offset after calibration
	* @param5: (output) calibration error
	* @param6: whether update the original tip position (default is false)
	* @return: pivot calibration alert
	*/
	uint16_t pivotTipCalibration(ToolCalibrationData& tool, std::vector<ToolTrackingData> trackingdataVect, std::vector<double>& tip, std::vector<double>& offset, double& error, bool originalTip = false, bool update = false);
	
	/**
	* @brief: fixed calibration for a tool tip
	* @param1: tool calibration data
	* @param2: current tracking data
	* @param3: tip position in the coordinate of the device
	* @param4: (output) calibrated tip in the coordinate of the tool
	* @param5: (output) tip offset after calibration
	* @param6: whether update the original tip position (default is false)
	* @return: fixed calibration alert
	*/
	uint16_t fixedTipCalibration(ToolCalibrationData& tool, ToolTrackingData trackingdata, MarkerPosition point, std::vector<double>& tip, std::vector<double>& offset, bool originalTip = false, bool update = false);

	/**
	* @brief: fixed calibration for a tool direction
	* @param1: tool calibration data
	* @param2: current tracking data
	* @param3: direction in the coordinate of the device
	* @param4: (output) calibrated direction in the coordinate of the tool
	* @param5: whether normalized the calibrated direction (default is true)
	* @param6: whether update the original direction (default is false)
	* @return: fixed calibration alert
	*/
	uint16_t fixedDirCalibration(ToolCalibrationData& tool, ToolTrackingData trackingdata, double vector[3], std::vector<double>& dir, bool normalize = true, bool update = false);

	/**
	* @brief: update tool after calibration
	* @param: new calibration information of the tool
	* @return: update result, if the input is not exist in the tool storage, return false
	* @warning: this function will not change the name of the tool in storage
	*/
	bool updateToolStorage(ToolCalibrationData tool);

	/**
	* @brief: get tool calibration data with the input name
	* @param: tool name
	* @return: tool calibration data. If the input tool name is not exist, the return is null 
	*/
	ToolCalibrationData getToolCalibrationData(std::string name);

	/**
	* @brief: project tracking data of a tool (source) to an another (target)
	* @param1: tracking data of the target tool
	* @param2: tracking data of the source tool
	* @return: tracking data of the projected source tool
	* @warning: if target or source tool is missing, the return is equal to the original tracking data of the source tool
	*/
	ToolTrackingData trackingDataProjection(ToolTrackingData targetTool, ToolTrackingData sourceTool);

	/**
	* @brief: the tracking data of the reference tool will be the first in the vector of all tracking data after setting, and call projected() for ToolTrackingData of other tool to get it tracking information in the coordinate of reference tool
	* @param: reference tool name
	* @return: true-success, false-failed(the input tool name is not exist)
	*/
	bool setReferenceTool(std::string toolName);

	/**
	* @brief: get the tracking volume of the connected device
	* @return: a 9D vector (near-x,y,z middle-x,y,z far-x,y,z) including near, middle and far plane extents of the tracking volume
	*/
	std::vector<int> getTrackingVolume();

	/**
	* @brief: judge the marker in/out tracking volume
	* @return: true-in, false-out
	*/
	bool markerInOutTrackingVolume(MarkerPosition p);

	/**
	* @brief: open or close laser
	* @param: open(true) or close(false)
	* @warning: this function only works for the devices of RT-PRO and RT-MAXV
	*/
	void laser(bool s);

	/**
	* @brief: open laser 
	* @param: delay time(ms)
	* @warning: this function only works for the devices of RT-PRO and RT-MAXV
	*/
	void laserOn(int delay);

	/**
	* @brief: beep
	* @param: delay time(ms)
	*/
	void beep(int delay);

	/**
	* @brief: get net adaptor information
	* @return: a struct of net adaptor information, including name, connection type, speed and connection status
	* @warning: the image transmission will be limited, if the speed is slower than 900MB/s
	*/
	NetAdaptorInfo getNetAdaptorInfo();

	/**
	* @brief: set device connection mode
	* @param1: true-automatic mode, false-fixed mode
	* @param2: IP addres for the fixed mode, default is null
	* @param3: netmask for the fixed mode, default is null
	* @param4: gateway for the fixed mode, default is null
	* @return: true-success, false-failed
	* @warning: setting IP to a fixed mode may cause connection error, and the mode will change after device restart
	*/
	bool setDeviceConnectionMode(bool autom, std::string IP = "", std::string MK = "", std::string GW = "");

	/**
	* @brief: send command to device
	* @param: command
	* @return: true-success, false-failed
	*/
	bool sendDeviceCommand(std::string command);

	/**
	* @brief: get log information
	* @return: log
	*/
	std::vector<std::string> getLog() { return log; };

	/**
	* @brief: save log information
	* @param: path
	* @return: true-success, false-failed
	*/
	bool saveLog(std::string path);

private:
	Tracking* TK;
	Tracking* VM;
	ToolRegi* ToolRegFilter;

	DeviceInfo RTdevice;
	std::vector<double> RCparameters;

	std::vector<ToolCalibrationData> trackTools;
	std::map<int, std::map<int, std::vector<int>>> neighbor;
	std::vector<std::vector<ToolPlaneMarker>> toolPlaneMarker;
	std::map<std::string, bool> trackingEnable;

	std::vector<MarkerPosition> allMarkersTrackingData;
	std::vector<MarkerPosition> unMatchedMarkers;
	
	std::vector<std::string> log;

	int iniExp;
	int expLeft;
	int expRight;

	bool resolution;
	char* leftimgDataforFrame;
	char* rightimgDataforFrame;
	unsigned char* videoDataforFrame;

	std::vector<std::vector<char>> reconstructDataVector;

	uint16_t transmissionStatus;
	uint16_t transmissionType;
	uint16_t reconstructionStatus;
	uint16_t connectionStatus;
	uint16_t systemAlert;
	uint16_t monitorStatus;

	std::vector<double> gravity{ 0.0, 0.0, 0.0 };
	double transMatrix[4][4];

	std::string referenceToolName;

	//generate tool registration set
	std::vector<ToolPlaneMarker> generateRegiSet(ToolCalibrationData tooldata);
	std::vector<ToolPlaneMarker> generateRegiSet(std::vector<std::vector<double>> D, std::vector<std::vector<double>> R, std::map<int, std::vector<int>> neighbor);

	//search calibration file
	void getFiles(const char * dir_name, std::string extend_name, std::vector<std::string>&file_names);
	
	//register tool markers to detected markers
	int toolReg(ToolCalibrationData tool, std::vector<ToolPlaneMarker> planeMarkers, std::vector<MarkerPosition> points, int index);

	//arom file generation filter condition
	bool getMarkersMinDis(std::vector<MarkerPosition> markerData, double radius);
	bool examineCharacter(std::string name);

	//optimize tool order
	void reorderToolStorage();

	//write to log
	void write2Log(std::string content);
};
#endif