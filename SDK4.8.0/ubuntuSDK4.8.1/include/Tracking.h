#ifndef TRACKING_HPP
#define TRACKING_HPP

#include "UdpReader.h"
#include "ToolRegi.h"
#include "SystemCRCTest.h"

#include <bitset>

class Tracking
{
public:
	Tracking();
	~Tracking();

	//initiation
	void init(std::string hostname, int port, bool autoIllum, bool errorPrint);

	//get connection statues
	int getConnectionStatues();
	void setRecTimeout(int t);

	//get connection IP
	std::vector<std::string> getIPs();

	//get connection MAC
	std::vector<std::string> getMACs();

	//get connection mask
	std::string getMask();

	//clear buffer
	void clearBuffer();

	//clear last command to initialize
	int initialize();

	//get parameter
	std::vector<double> getParameter();

	//start and end tracking
	int startTraking();
	int endTracking();

	//start and end imaging
	int startImaging();
	int endImaging();
	void setImageLuminance(double l, int side);
	void setImageContrast(double c, int side);

	//start and end video monitor
	int startVideoMonitor(bool resolution = true);
	int endVideoMonitor();

	//beep
	int beep(int t);

	//start and end reconstructing
	int startRecImaging();
	int endRecImaging();
	void setMEMSparams(int material = 0);
	std::vector<int> getMEMSparams(int level);
	void updateDistStage3D(int level);
	void setReconstructArea(short width, short height);
	int startReconstructing(short leftX, short leftY, short rightX, short rightY, int type = 0);
	int endReconstructing();
	int reconstructingReady();
	int texturedReconstructingReady();

	//update tracking, reconstruction and monitor information
	int trackingBX(std::vector<std::string>& warnMessage, std::vector<MarkerPosition>& Cordis, uint16_t transmissionType, char* leftimg, char* rightimg, int version);
	int receiveReconstructImages(int comm, std::vector<char> &leftimg, std::vector<char> &rightimg);
	int receiveReconstructVisualImage(int comm, std::vector<char> &img);
	int receiveMonitorImages(unsigned char* img, bool resolution);

	//read firmware version
	std::vector<int> readFirmwareVersion();

	//send command to device
	int sendCommand(std::string command) const;
	int sendAN5642Command(char* command) const;

	//display area on the images
	void drawArea(short width, short height, short leftX, short leftY, short rightX, short rightY);
	void hideArea();

	//check ingormation
	void systemCheck(std::string& warnMsg);
	void messageCheck(std::string& msg);

	//close connection
	int closeUDP();

private:
	bool autoExposure;

	std::vector<double> luminance, contrast;

	bool areaDisplay;
	std::vector<int> displayArea;

	int MEMSparams[9][2];

	int reconstructAreaWidth;
	int reconstructAreaHeight;

	UdpConnection* connection_;
	SystemCRCTest* crcValidator_;
	UdpReader reader;
};
#endif TRACKING_HPP