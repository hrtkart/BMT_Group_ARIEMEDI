#include "monitoringWorker.h"

monitoringWorker::monitoringWorker(ARMDCombinedAPI* Tracker)
{
	m_Tracker = Tracker;
	centerImage = new unsigned char[1920 * 1200 * 3];
}

monitoringWorker::~monitoringWorker()
{
	if (centerImage != nullptr)
	{
		delete[] centerImage;
		centerImage = nullptr;
	}
}

void monitoringWorker::setThreadStatus(int status)
{
	m_status = status;
	if (!m_status)
	{
		m_Tracker->stopVideoMonitor();
	}
}

void monitoringWorker::resolutionChange(bool level)
{
	m_status = false;
	Sleep(50);
	m_Tracker->setVLImageResolution(level);
}

void monitoringWorker::run()
{
	int width = 0, height = 0;
	while (m_status)
	{
		//update monitoring data 
		m_Tracker->monitoringUpdate();

		//interuption check
		if (ConnectionStatus::Interruption == m_Tracker->getConnectionStatus())
		{
			qDebug() << "Interruption! Please check connection!";
			m_status = false;
			continue;
		}
		width = m_Tracker->getVLImageSize().at(0);
		height = m_Tracker->getVLImageSize().at(1);

		memcpy(centerImage, m_Tracker->getVideoMonitorData(), static_cast<size_t>(width * height * 3));

		emit SendMonitoringInformation(centerImage, width, height);
	}
}