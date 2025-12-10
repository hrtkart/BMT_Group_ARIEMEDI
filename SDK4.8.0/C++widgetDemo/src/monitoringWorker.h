#ifndef MONITORINGWORKER_H
#define MONITORINGWORKER_H

#include "ARMDCombinedAPI.h"

#include <QtCore/qthread.h>
#include <QDebug>
#include <qimage.h>

//class ARMDCombinedAPI;
class monitoringWorker : public QThread
{
	Q_OBJECT
public:
	monitoringWorker(ARMDCombinedAPI* Tracker);
	~monitoringWorker();

	//set thread status
	void setThreadStatus(int status);

	void resolutionChange(bool level);

protected:
	void run();

signals:
	//send monitoring data
	void SendMonitoringInformation(unsigned char*, int, int);
	
private:
	ARMDCombinedAPI* m_Tracker;
	int m_status = 0;
	unsigned char* centerImage;
};
#endif // ! MONITORINGWORKER_H