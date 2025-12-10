#ifndef TRACKINGWORKER_H
#define TRACKINGWORKER_H

#include "ARMDCombinedAPI.h"
#include "DeviceScan.h"
#include "ReconstructPointCloud.h"

#include <QtCore/qthread.h>
#include <qimage.h>
#include <QDebug>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <QPixmap.h>

class trackingWorker : public QThread
{
	Q_OBJECT
public:
	trackingWorker(ARMDCombinedAPI* tracker);
	~trackingWorker();

	//connect and disconnect RT
	int autoConnect();
	int connect(std::string hostname);
	void disconnect();
	std::vector<int> printAPIversion();
	uint16_t getDeviceType();

	//load tool information
	void loadPassiveToolAROM(std::string path);
	void loadActiveWirelessToolAROM(std::string path);
	void loadActiveToolAROM(std::string path);

	//get tool information
	std::vector<ToolCalibrationData> getToolStorage();
	int getTrackToolsNum();
	std::vector<std::string> getToolName();

	//start and stop tracking
	void startTracking();
	void stopTracking();

	//start and stop imaging
	void startImaging();
	void stopImaging();
	void setLuminance(double l);
	void setContrast(double c);
	void setAreaDisplayOff();
	void setAreaDisplay(short width, short height, short leftX, short leftY, short rightX, short rightY);

	//reconstruction
	void reconstructPointCloud(short width, short height, short leftX, short leftY, short rightX, short rightY);
	
	//start and stop monitoring
	void startMonitoring();
	void stopMonitoring();

	//update tracking and imaging data
	std::vector<MarkerPosition> getAllMarkers();
	std::vector<MarkerPosition> getUnMatchedMarkers();

	//get system status
	uint16_t getConnectionStatus();
	uint16_t getTransmissionStatus();
	uint16_t getSystemAlert();
	
	//create an arom file
	void generateAROM(std::string directory, ToolCalibrationData tool);

	std::vector<ToolTrackingData> getTrackingData(std::vector<MarkerPosition> cordi);

	//set thread status
	void setThreadStatus(int status);

	//toolbox
	uint16_t fixedTipCalibration(ToolCalibrationData& tool, ToolTrackingData trackingdata, MarkerPosition point, std::vector<double>& tip, std::vector<double>& offset, bool originalTip = false, bool update = false);
	uint16_t fixedDirCalibration(ToolCalibrationData& tool, ToolTrackingData trackingdata, double vector[3], std::vector<double>& dir, bool normalize = false, bool update = false);
	void laserOnOff(bool status);
	void beep(int t);

protected:
	void run();

signals:
	//send tracking data
	void SendTrackingInformation(std::vector<ToolTrackingData>);
	void SendUnMatchedMarkers(std::vector<MarkerPosition>);
	void SendAllMarkers(std::vector<MarkerPosition>);

	//send imaging data
	void SendImagingInformation(char*, char*, int, int);
	
private:
	ARMDCombinedAPI* m_Tracker;
	int m_status = 0;

	bool m_isResTruct = true;
	ReconstructPointCloud* m_Reconstructor;
};
#endif // ! TRACKINGWORKER