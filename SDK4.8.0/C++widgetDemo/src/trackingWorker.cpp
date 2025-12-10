#include "trackingWorker.h"
#include <QTime>

trackingWorker::trackingWorker(ARMDCombinedAPI* tracker)
{
	m_Tracker = tracker;
}

trackingWorker::~trackingWorker()
{
}

int trackingWorker::autoConnect()
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
		IP = x.second;

		std::cout << "The firt scaned RT device hostname is " << hostname << ", and IP is " << IP << std::endl;
		break;
	}

	return m_Tracker->connect(hostname);
}

int trackingWorker::connect(std::string hostname)
{
	return m_Tracker->connect(hostname);
}

void trackingWorker::disconnect()
{
	m_Tracker->disconnect();
}

uint16_t trackingWorker::getConnectionStatus()
{
	return m_Tracker->getConnectionStatus();
}

std::vector<int> trackingWorker::printAPIversion()
{
	return m_Tracker->printAPIandFirmwareVersion();
}

uint16_t trackingWorker::getDeviceType()
{
	return m_Tracker->getConnectedDeviceInfo().deviceType;
}

void trackingWorker::beep(int t)
{
	return m_Tracker->beep(t);
}

uint16_t trackingWorker::getTransmissionStatus()
{
	return m_Tracker->getTransmissionStatus();
}

uint16_t trackingWorker::getSystemAlert()
{
	return m_Tracker->getSystemAlert();
}

void trackingWorker::startTracking()
{
	m_Tracker->startTracking();
	m_Tracker->setTrackingDataTransmissionType(4);
}

void trackingWorker::stopTracking()
{
	m_Tracker->stopTracking();
}

void trackingWorker::setLuminance(double l)
{
	m_Tracker->setIFLuminance(l);
}

void trackingWorker::setContrast(double c)
{
	m_Tracker->setIFContrast(c);
}

void trackingWorker::setAreaDisplay(short width, short height, short leftX, short leftY, short rightX, short rightY)
{
	m_Tracker->setAreaDisplay(width, height, leftX, leftY, rightX, rightY);
}

void trackingWorker::reconstructPointCloud(short width, short height, short leftX, short leftY, short rightX, short rightY)
{
	if (m_isResTruct)
	{
		m_Tracker->reconstructPointCloud(width, height, leftX, leftY, rightX, rightY);

		m_isResTruct = false;
	}
}

void trackingWorker::setAreaDisplayOff()
{
	m_Tracker->setAreaHidden();
}

void trackingWorker::startImaging()
{
	m_Tracker->startImaging();
	m_Tracker->setImagingMode(0);
}

void trackingWorker::startMonitoring()
{
	m_Tracker->startVideoMonitor(true);
}

void trackingWorker::stopImaging()
{
	m_Tracker->stopImaging();
}

void trackingWorker::stopMonitoring()
{
	m_Tracker->stopVideoMonitor();
}

void trackingWorker::loadPassiveToolAROM(std::string path)
{
	m_Tracker->loadPassiveToolAROM(path);
}

void trackingWorker::loadActiveWirelessToolAROM(std::string path)
{
	m_Tracker->loadActiveWirelessToolAROM(path);
}

void trackingWorker::loadActiveToolAROM(std::string path)
{
	m_Tracker->loadActiveToolAROM(path);
}

std::vector<ToolCalibrationData> trackingWorker::getToolStorage()
{
	return m_Tracker->getToolStorage();
}

int trackingWorker::getTrackToolsNum()
{
	return m_Tracker->getTrackToolsNum();
}

std::vector<std::string> trackingWorker::getToolName()
{
	return std::vector<std::string>();
}

void trackingWorker::setThreadStatus(int status)
{
	m_status = status;
}

uint16_t trackingWorker::fixedTipCalibration(ToolCalibrationData& tool, ToolTrackingData trackingdata, MarkerPosition point, std::vector<double>& tip, std::vector<double>& offset, bool originalTip, bool update)
{
	return m_Tracker->fixedTipCalibration(tool, trackingdata, point, tip, offset, originalTip, update);
}

uint16_t trackingWorker::fixedDirCalibration(ToolCalibrationData& tool, ToolTrackingData trackingdata, double vector[3], std::vector<double>& dir, bool normalize, bool update)
{
	return m_Tracker->fixedDirCalibration(tool, trackingdata, vector, dir, normalize, update);
}

void trackingWorker::laserOnOff(bool status)
{
	m_Tracker->laserOn(status);
}

std::vector<MarkerPosition> trackingWorker::getAllMarkers()
{
	return m_Tracker->getAllMarkers();
}

std::vector<MarkerPosition> trackingWorker::getUnMatchedMarkers()
{
	return m_Tracker->getUnMatchedMarkers();
}

void trackingWorker::generateAROM(std::string directory, ToolCalibrationData tool)
{
	m_Tracker->generateAROM(directory, tool);
}

std::vector<ToolTrackingData> trackingWorker::getTrackingData(std::vector<MarkerPosition> cordi)
{
	return m_Tracker->getTrackingData(cordi);
}

void trackingWorker::run()
{
	char* leftImage = new char[m_Tracker->getIFImageSize().at(0) * m_Tracker->getIFImageSize().at(1)];
	char* rightImage = new char[m_Tracker->getIFImageSize().at(0) * m_Tracker->getIFImageSize().at(1)];

	while (m_status)
	{
		//update tracking and imaging data 
		int flag = m_Tracker->trackingUpdate();

		//interuption check
		if (ConnectionStatus::Interruption == m_Tracker->getConnectionStatus())
		{
			qDebug() << "Interruption! Please check connection!";
			m_status = false;
			continue;
		}

		//send current tracking and imaging data
		emit SendAllMarkers(m_Tracker->getAllMarkers());

		#ifdef NDEBUG
			memcpy(leftImage, m_Tracker->getLeftImagingData(), static_cast<size_t>((int)m_Tracker->getIFImageSize().at(0) * m_Tracker->getIFImageSize().at(1)));
			memcpy(rightImage, m_Tracker->getRightImagingData(), static_cast<size_t>((int)m_Tracker->getIFImageSize().at(0) * m_Tracker->getIFImageSize().at(1)));
			emit SendImagingInformation(leftImage, rightImage, m_Tracker->getIFImageSize().at(0), m_Tracker->getIFImageSize().at(1));
		#else
			if (flag == 2)
			{
				memcpy(leftImage, m_Tracker->getLeftImagingData(), static_cast<size_t>((int)m_Tracker->getIFImageSize().at(0) * m_Tracker->getIFImageSize().at(1)));
				memcpy(rightImage, m_Tracker->getRightImagingData(), static_cast<size_t>((int)m_Tracker->getIFImageSize().at(0) * m_Tracker->getIFImageSize().at(1)));
				emit SendImagingInformation(leftImage, rightImage, m_Tracker->getIFImageSize().at(0), m_Tracker->getIFImageSize().at(1));
			}
		#endif	

		//reconstruction only for PRO
		if (m_Tracker->getReconstructionStatus() == TransmissionStatus::ReconstructionReady)
		{
			m_Reconstructor = new ReconstructPointCloud();
			m_Reconstructor->initializeParameters(m_Tracker->getReconstructParameters());
			m_Reconstructor->setReconstructData(m_Tracker->getReconstructionData());

			PointCloudInfo pc = m_Reconstructor->getPointCloud(true);
			if (pc.num > 0)
				m_Reconstructor->savePLY("./pc.ply", pc, true);
			std::cout << "PointSize:" << pc.num << std::endl;

			cv::imwrite("./Texture.bmp", m_Reconstructor->getTextureImageMat());
			std::vector<int> clippedPCindex= m_Reconstructor->project2DImageRectArea(700,580,100,120);

			PointCloudInfo clippedPC;
			clippedPC.num = 0;
			for (int i = 0; i < clippedPCindex.size(); i++)
			{
				clippedPC.insertPoint(pc.points.at(clippedPCindex.at(i)));
			}
			m_Reconstructor->savePLY("./clippedpc.ply", clippedPC, true);
			
			m_Tracker->beep(40);
			m_isResTruct = true;

			if (m_Reconstructor != nullptr)
			{
				delete m_Reconstructor;
				m_Reconstructor = nullptr;
			}
		}
	}

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