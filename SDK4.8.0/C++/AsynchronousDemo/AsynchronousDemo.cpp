//RT SDK header
#include "ARMDCombinedAPI.h"
#include "DeviceScan.h"
#include "ReconstructPointCloud.h"

//other headers
#include <time.h>
#include <thread>

using namespace std;

static ARMDCombinedAPI m_Tracker = ARMDCombinedAPI();
bool updateFlag = false;
short width = 360;
short height = 363;
short leftX = 776;
short leftY = 418;
short rightX = 776;
short rightY = 418;
bool quality = true;

//generate arom
void generateAROMfile()
{
	MarkerPlane plane;
	MarkerPosition a;
	a.P[0] = 0.00;
	a.P[1] = 0.00;
	a.P[2] = 0.00;
	a.P[3] = 1.00;
	MarkerPosition b;
	b.P[0] = 0.00;
	b.P[1] = -50.00;
	b.P[2] = 0.00;
	b.P[3] = 1.00;
	MarkerPosition c;
	c.P[0] = -25.00;
	c.P[1] = -100.00;
	c.P[2] = 0.00;
	c.P[3] = 1.00;
	MarkerPosition d;
	d.P[0] = 25.00;
	d.P[1] = -135.00;
	d.P[2] = 0.00;
	d.P[3] = 1.00;
	plane.markers.push_back(a);
	plane.markers.push_back(b);
	plane.markers.push_back(c);
	plane.markers.push_back(d);

	ToolCalibrationData newTool;
	newTool.name = "Tool";
	newTool.markerType = 0;
	newTool.planeNum = 1;
	newTool.minNumMarker = 3;
	newTool.calbError = -1;//no calibrated tip, the tip will be set to (0,0,0)
	newTool.planes.push_back(plane);
	newTool.algorithmType = 1;
	//newTool.pin[0] = -0.08;
	//newTool.pin[1] = 158.26;
	//newTool.pin[2] = -18.21;

	//generate tool
	cout << GenerationStatus::toString(m_Tracker.generateAROM("./tool/", newTool)) << endl;
}

//convert to arom
void convert2AROMfile()
{
	cout << GenerationStatus::toString(m_Tracker.convert2AROM("./tool/", "./tool/Tool.rom")) << endl;
}

//connect system
int connect(string hostname)
{
	cout << "The device " << hostname << " is to be connected." << endl;

	cout << "Start connection......" << endl;
	int errorCode = m_Tracker.connect(hostname);
	if (errorCode == 0)
	{
		cout << "Successed!" << endl;
		cout << "RT device firmware version is " << m_Tracker.getConnectedDeviceInfo().firmwareVersion[0] 
			                               <<"." << m_Tracker.getConnectedDeviceInfo().firmwareVersion[1]
			                               <<"." << m_Tracker.getConnectedDeviceInfo().firmwareVersion[2] <<endl;
		cout << "RT device IP address is " << m_Tracker.getConnectedDeviceInfo().IP << endl;
		cout << "Local IP address is " << m_Tracker.getNetAdaptorInfo().IP << endl;
		cout << "Network speed is " << m_Tracker.getNetAdaptorInfo().speed << "MB/s" << endl;
		cout << "Net adaptor name is " << m_Tracker.getNetAdaptorInfo().name << endl;
		cout << "Connection type is " << m_Tracker.getNetAdaptorInfo().connectionType << endl;

		updateFlag = true;
		return 0;
	}
	else
	{
		//print connection error information
		cout << ConnectionStatus::toString(m_Tracker.getConnectionStatus()) << endl;

		cout << "Failed!" << endl;
		return -1;
	}
}

//close system
void closeSystem()
{
	updateFlag = false;
	cout << "Close connection......";
	if (ConnectionStatus::Interruption == m_Tracker.getConnectionStatus())
	{
		m_Tracker.disconnect();

		cout << "Succesed！" << endl;
	}
	else
	{
		//wait for thread finish
		Sleep(1000);

		//stop data transmission and disconnect device
		m_Tracker.stopTracking();
		m_Tracker.stopImaging();
		m_Tracker.disconnect();

		if (ConnectionStatus::DisConnected == m_Tracker.getConnectionStatus())
			cout << "Succesed！" << endl;
		else
			cout << "Failed!" << endl;
	}
}

//print tracking information
void printToolData(ToolTrackingData toolData)
{
	//show tool information
	cout << "tool name:" << toolData.name.c_str() << endl;

	//show transformation status
	cout << "tool transformation status: " << TransformationStatus::toString(toolData.transform.status) << endl;

	if (toolData.matchStatus)
	{
		//show position of each marker on tool
		vector<MarkerPosition> Cordi = toolData.plane.markers;
		cout << "position of each marker: " << endl;
		for (int i = 0; i < Cordi.size(); i++)
			cout << Cordi[i].P[0] << "," << Cordi[i].P[1] << "," << Cordi[i].P[2] << endl;

		//show acquistion time
		string atime = toolData.timespec;
		cout << "acquisition time: " << atime << endl;

		//show matching error
		double rms = toolData.error;
		cout << "matching error: " << rms << endl;

		//show matching plane
		int planeIndex = toolData.matchedPlane;
		cout << "matching plane: " << planeIndex << endl;

		//show transformation matrix, Euler angle and quaternion 
		double matr[4][4];
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				matr[i][j] = toolData.transform.matrix[i][j];

		cout << "transformation matrix: " << "\n" << matr[0][0] << "," << matr[0][1] << "," << matr[0][2] << "," << matr[0][3] << "\n"
			<< matr[1][0] << "," << matr[1][1] << "," << matr[1][2] << "," << matr[1][3] << "\n"
			<< matr[2][0] << "," << matr[2][1] << "," << matr[2][2] << "," << matr[2][3] << "\n"
			<< matr[3][0] << "," << matr[3][1] << "," << matr[3][2] << "," << matr[3][3] << endl;

		cout << "Euler angle: " << toolData.transform.yaw << " " << toolData.transform.pitch << " " << toolData.transform.roll << endl;

		cout << "quaternion: " << toolData.transform.qw << " " << toolData.transform.qx << " " << toolData.transform.qy << " " << toolData.transform.qz << endl;

		//show tip position
		double tip[3];
		tip[0] = toolData.transform.tx, tip[1] = toolData.transform.ty, tip[2] = toolData.transform.tz;
		cout << "tip position: " << tip[0] << "," << tip[1] << "," << tip[2] << endl;
	}

	cout << "-------------------------------------------------------------------------" << endl;
}

//load tool
void configureTools()
{
	m_Tracker.loadPassiveToolAROM("./tool");
}

void trackingUpdate()
{
	while (updateFlag)
	{
		//update tracking and imaging data 
		m_Tracker.trackingUpdate();

		//interuption check
		if (ConnectionStatus::Interruption == m_Tracker.getConnectionStatus())
		{
			updateFlag = false;
			cout << "Connection is interrupted!" << endl;
			return;
		}

		//reconstruction
		if (m_Tracker.getReconstructionStatus() == TransmissionStatus::ReconstructionReady)
		{
			ReconstructPointCloud Reconstructor = ReconstructPointCloud();
			Reconstructor.initializeParameters(m_Tracker.getReconstructParameters());
			Reconstructor.setReconstructData(m_Tracker.getReconstructionData());
			PointCloudInfo pc = Reconstructor.getPointCloud(true);

			//get depth image
			//cv::Mat Zchannel;
			//cv::Mat ZchannelGray;
			//std::vector<cv::Mat> channels(3);
			//cv::split(Reconstructor.getDepthImage(), channels);
			//Zchannel = channels[2];
			//Zchannel.convertTo(ZchannelGray, CV_8U, 255.0 / 3000);
			//cv::imwrite("./depthImageZ.bmp", ZchannelGray);

			//save reconstructed point cloud
			Reconstructor.savePLY("./pc.ply", pc, true);
			vector<float> center = pc.getBoundCenter();
			vector<float> bound = pc.getBoundingBox();
			vector<float> gcenter = pc.getGeometricalCenter();
			cout << "the pointSize: " << pc.num << " num." << endl;
			m_Tracker.beep(40);

			//validate the quality of the point cloud
			bool currentQuality = m_Tracker.evaluateReconstruction(pc.stage);

			//the low quality may be improved by second reconstruction, double-reconstruction consume more time than single-reconstruction
			//double-reconstruction if you facus on quality, single-reconstruction if you facus on efficiency
			/*if (!currentQuality && currentQuality != quality)
			{
				quality = currentQuality;
				m_Tracker.reconstructPointCloud(width, height, leftX, leftY, rightX, rightY);
			}
			else
			{
				Reconstructor.savePLY("./pc.ply", pc, false);
				cout << "the pointSize: " << pc.num << " num." << endl;
				m_Tracker.beep(40);
			}*/
		}
	}
}

//print tracking and imaging information
void printTrackingImagingData()
{
	//start tracking and imaging
	m_Tracker.startTracking();
	m_Tracker.startImaging();
	vector<int> imageSize = m_Tracker.getIFImageSize();

	for (int ite = 0; ite < 10000; ite++)
	{
		//show system alert information
		if (DeviceAlert::Normal != m_Tracker.getSystemAlert())
			cout << DeviceAlert::toString(m_Tracker.getSystemAlert()) << endl;

		//interuption check
		if (ConnectionStatus::Interruption == m_Tracker.getConnectionStatus())
		{
			updateFlag = false;
			cout << "Connection is interrupted!" << endl;
			return;
		}

		vector<double> gravity = m_Tracker.getGravityVector();
		cout << "Gravity vector is " << gravity.at(0) << " " << gravity.at(1) << " " << gravity.at(2) << endl;

		char* leftImage = new char[imageSize.at(0) * imageSize.at(1)];
		char* rightImage = new char[imageSize.at(0) * imageSize.at(1)];
		memcpy(leftImage, m_Tracker.getLeftImagingData(), imageSize.at(0) * imageSize.at(1) * sizeof(char));
		memcpy(rightImage, m_Tracker.getRightImagingData(), imageSize.at(0) * imageSize.at(1) * sizeof(char));

		//print tracking data
		cout << "All passitive markers number is " << m_Tracker.getAllMarkers(1).size() << endl;
		cout << "All active markers number is " << m_Tracker.getAllMarkers(2).size() << endl;
		cout << "All markers number is " << m_Tracker.getAllMarkers(0).size() << endl;

		//the size of toolData corresponds to the number of input tool
		vector<ToolTrackingData> toolData = m_Tracker.getTrackingData(m_Tracker.getAllMarkers(0));
		for (int i = 0; i < toolData.size(); i++)
			printToolData(toolData[i]);

		cout << "Stray markers number is " << m_Tracker.getUnMatchedMarkers().size() << endl;

		if (leftImage != nullptr)
		{
			delete[] leftImage;
			leftImage = nullptr;
		}
		if (rightImage != nullptr)
		{
			delete[] rightImage;
			rightImage = nullptr;
		}
	}
}

void reconstructPointCloud()
{
	quality = true;
	m_Tracker.setMaterial(1);
	m_Tracker.reconstructPointCloud(width, height, leftX, leftY, rightX, rightY);

	while (updateFlag)
	{
		int wait = 0;
	}
}

//set tracking data transmission type
void setTrackingDataTransmissionType(int type)
{
	switch (type)
	{
	case 0:
		m_Tracker.setTrackingDataTransmissionType(TransmissionType::Passive); 
		break;
	case 1:
		m_Tracker.setTrackingDataTransmissionType(TransmissionType::Active);
		break;
	case 2:
		m_Tracker.setTrackingDataTransmissionType(TransmissionType::Mixed);
		break;
	case 3:
		m_Tracker.setTrackingDataTransmissionType(TransmissionType::Alternate);
		break;
	default:
		break;
	}
}

int main()
{
	//manually set hostname and connect RT device
	//string hostname = "RT-MAX000001.local";

	//auto scan RT device, and connect the first scaned device
	string hostname;
	string IP;
	DeviceScan *deviceScan = new DeviceScan();
	while (deviceScan->getDeviceInfo().size() == 0)
	{
		deviceScan->updateDeviceInfo();
	}

	for (auto& x : deviceScan->getDeviceInfo())
	{
		hostname = x.first;
		IP = x.second;

		cout << "The firt scaned RT device hostname is " << hostname << ", and IP is " << IP << endl;
		break;
	}

	//generate a testing standard arom file
	generateAROMfile();

	//convert a non-arom file to standard arom file
	//convert2AROMfile();

	//hostname or IP of the device to be connected
	if (connect(hostname) != 0)
	{
		cout << "Connection failed!" << endl;
		cout << "Press Enter to continue...";
		cin.ignore();
		return -1;
	}

	//load all .arom files from the input path
	configureTools();

	//set tracking data transmission type
	setTrackingDataTransmissionType(0);

	thread armdThread(trackingUpdate);
	armdThread.detach();
	
	//print tracking information
	printTrackingImagingData();

	//reconstruct point cloud
	//reconstructPointCloud();

	//close tracking system
	closeSystem();

	//print log information
	std::vector<std::string> log = m_Tracker.getLog();
	for (int i = 0; i < log.size(); i++)
		std::cout << log.at(i) << std::endl;
	m_Tracker.saveLog("./record.log");

	return 0;
}