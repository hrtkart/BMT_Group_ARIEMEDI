//RT SDK header
#include "ARMDCombinedAPI.h"
#include "DeviceScan.h"
#include "ReconstructPointCloud.h"

#include <thread>

using namespace std;

ARMDCombinedAPI* m_Tracker = new ARMDCombinedAPI();
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
	a.P[0] = 0.0;
	a.P[1] = 0.0;
	a.P[2] = 0.0;
	a.P[3] = 1.0;
	MarkerPosition b;
	b.P[0] = 0.0;
	b.P[1] = -50.0;
	b.P[2] = 0.0;
	b.P[3] = 1.0;
	MarkerPosition c;
	c.P[0] = -25.0;
	c.P[1] = -100.0;
	c.P[2] = 0.0;
	c.P[3] = 1.0;
	MarkerPosition d;
	d.P[0] = 25.0;
	d.P[1] = -135.0;
	d.P[2] = 0.0;
	d.P[3] = 1.0;
	plane.markers.push_back(a);
	plane.markers.push_back(b);
	plane.markers.push_back(c);
	plane.markers.push_back(d);

	ToolCalibrationData newTool;
	newTool.name = "Tool";
	newTool.markerType = 0;
	newTool.planeNum = 1;
	newTool.minNumMarker = 3;
	newTool.calbError = -1;
	newTool.planes.push_back(plane);
	newTool.pin[0] = -0.08;
	newTool.pin[0] = 158.26;
	newTool.pin[0] = -18.21;

	//generate tool
	cout << GenerationStatus::toString(m_Tracker->generateAROM("./tool/", newTool)) << endl;
}

//connect system
int connect(string hostname)
{
	cout << "The device " << hostname << " is to be connected." << endl;

	cout << "Start connection......" << endl;
	int errorCode = m_Tracker->connect(hostname);
	if (errorCode == 0)
	{
		cout << "Successed!" << endl;
		std::cout << "Local IP address is " << m_Tracker->getNetAdaptorInfo().IP << std::endl;
		std::cout << "Local subnet mask is " << m_Tracker->getNetAdaptorInfo().mask << std::endl;
        std::cout << "RT device IP address is " << m_Tracker->getConnectedDeviceInfo().IP << std::endl;
		updateFlag = true;
		return 0;
	}
	else
	{
		//print connection error information
		cout << ConnectionStatus::toString(m_Tracker->getConnectionStatus()) << endl;

		cout << "Failed!" << endl;
		return -1;
	}
}

//close system
void closeSystem()
{
	updateFlag = false;
	cout << "Close connection......";
	if (ConnectionStatus::Interruption == m_Tracker->getConnectionStatus())
	{
		m_Tracker->disconnect();

		cout << "Succesed！" << endl;
		delete m_Tracker;
	}
	else
	{
		m_Tracker->stopTracking();
		m_Tracker->stopImaging();
		m_Tracker->disconnect();

		if (ConnectionStatus::DisConnected == m_Tracker->getConnectionStatus())
		{
			cout << "Succesed！" << endl;
			delete m_Tracker;
		}
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
		for (int j = 0; j < Cordi.size(); j++)
			cout << Cordi[j].P[0] << "," << Cordi[j].P[1] << "," << Cordi[j].P[2] << endl;

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
		for (int j = 0; j < 4; j++)
			for (int k = 0; k < 4; k++)
				matr[j][k] = toolData.transform.matrix[j][k];
		
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
	m_Tracker->loadPassiveToolAROM("./tool");
}

//
void trackingUpdate()
{
	while (updateFlag)
	{
		//update tracking data 
		m_Tracker->trackingUpdate();
		// cout<<m_Tracker->getAllMarkers().size()<<endl;
		//reconstruction
		if (m_Tracker->getReconstructionStatus() == TransmissionStatus::ReconstructionReady)
		{
			ReconstructPointCloud Reconstructor = ReconstructPointCloud();
			Reconstructor.initializeParameters(m_Tracker->getReconstructParameters());
			Reconstructor.setReconstructData(m_Tracker->getReconstructionData());
			PointCloudInfo pc = Reconstructor.getPointCloud(true);

			//save reconstructed point cloud
			Reconstructor.savePLY("./pc.ply", pc , true);
			cout << "the point size is " << pc.num << endl;
			m_Tracker->beep(40);

			//validate the quality of the int cloud
			bool currentQuality = m_Tracker->evaluateReconstruction(pc.stage);
		}
	}
}

//print tracking information
void printTrackingData()
{
	//start tracking
	m_Tracker->startTracking();
	m_Tracker->startImaging();
	std::vector<int> imageSize = m_Tracker->getIFImageSize();

	for (int ite = 0; ite < 10000; ite++)
	{
		//show system alert information
		if (DeviceAlert::Normal != m_Tracker->getSystemAlert())
			cout << DeviceAlert::toString(m_Tracker->getSystemAlert()) << endl;

		//interuption check
		if (ConnectionStatus::Interruption == m_Tracker->getConnectionStatus())
		{
			updateFlag = false;
			cout << ConnectionStatus::toString(m_Tracker->getConnectionStatus()) << endl;
			return;
		}

		std::vector<double> gravity = m_Tracker->getGravityVector();
		cout << "Gravity vector is " << gravity.at(0) << " " << gravity.at(1) << " " << gravity.at(0) << endl;

		char* leftImage = new char[1280 * 800];
		char* rightImage = new char[1280 * 800];
		memcpy(leftImage, m_Tracker->getLeftImagingData(), 1280 * 800 * sizeof(char));
		memcpy(rightImage, m_Tracker->getRightImagingData(), 1280 * 800 * sizeof(char));

		//the size of toolData corresponds to the number of input tool
		vector<MarkerPosition> allMarkerData = m_Tracker->getAllMarkers();
		vector<ToolTrackingData> toolData = m_Tracker->getTrackingData(allMarkerData);
		
		cout << "markers number:" << m_Tracker->getAllMarkers(1).size() << endl;
		
		for (int i = 0; i < toolData.size(); i++)
			printToolData(toolData[i]);

		cout << "Stray markers number is " << m_Tracker->getUnMatchedMarkers().size() << endl;

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

//reconstruct point cloud
void reconstructPointCloud()
{
	quality = true;
	m_Tracker->setMaterial(1);
	m_Tracker->reconstructPointCloud(width, height, leftX, leftY, rightX, rightY);

	while (true)
	{
		int wait =0;
	}
}

//set tracking date transmission type
void setTrackingDataTransmissionType(int type)
{
	switch (type)
	{
	case 0:
		m_Tracker->setTrackingDataTransmissionType(TransmissionType::Passive);
		break;
	case 1:
		m_Tracker->setTrackingDataTransmissionType(TransmissionType::Active);
		break;
	case 2:
		m_Tracker->setTrackingDataTransmissionType(TransmissionType::Mixed);
		break;
	case 3:
		m_Tracker->setTrackingDataTransmissionType(TransmissionType::Alternate);
		break;
	default:
		break;
	}
}

int main()
{
	std::string hostname;
	std::string IP;
	DeviceScan* deviceScan = new DeviceScan();
	while (deviceScan->getDeviceInfo().size() == 0)
	{
		deviceScan->updateDeviceInfo();
	}

	for (auto& x : deviceScan->getDeviceInfo())
	{
		hostname = x.first;
		IP = x.second.at(0);

		std::cout<<"The first scanned RT device hostname is " << hostname << ", and IP is " << IP << std::endl;
		break;
	}

	//hostname or IP of the device to be connected
	// string hostname = "RT-PRO300113.local";

	//generate a testing standard arom file
	generateAROMfile();

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

	std::thread armdThread(trackingUpdate);
	armdThread.detach();

	//print tracking information
	printTrackingData();

	//reconstruct point cloud
	//reconstructPointCloud();

	//close tracking system
	closeSystem();

	return 0;
}