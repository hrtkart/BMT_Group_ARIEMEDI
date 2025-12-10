#include "widget_Demo.h"

widget_Demo::widget_Demo(QWidget* parent)
	: QMainWindow(parent)
{
	//setup UI
	ui.setupUi(this);

	//register signal
	qRegisterMetaType<const char*>("const char*");
	qRegisterMetaType<std::vector<ToolTrackingData>>("std::vector<ToolTrackingData>");
	qRegisterMetaType<std::vector<MarkerPosition>>("std::vector<MarkerPosition>");
	qRegisterMetaType<std::vector<char>>("std::vector<char>");

	m_Tracker = new ARMDCombinedAPI();

	//creat trackingworker
	m_Trackerthread = new trackingWorker(m_Tracker);
	QObject::connect(m_Trackerthread, SIGNAL(SendImagingInformation(char*, char*, int, int)), this, SLOT(OnUpdateImaging(char*, char*, int, int)));
	QObject::connect(m_Trackerthread, SIGNAL(SendAllMarkers(std::vector<MarkerPosition>)), this, SLOT(OnUpdateAllMarkers(std::vector<MarkerPosition>)));
	
	//button connection
	QObject::connect(ui.CapturePhotographBtn, SIGNAL(clicked()), this, SLOT(OnCapturePhotograph()));
	QObject::connect(ui.ptCloudBtn, SIGNAL(clicked()), this, SLOT(OnPtCloudBtn()));

	//slider connection
	QObject::connect(ui.LuminanceSlider, SIGNAL(valueChanged(int)), this, SLOT(OnLuminanceChange(int)));
	QObject::connect(ui.ContrastSlider, SIGNAL(valueChanged(int)), this, SLOT(OnContrastChange(int)));

	//inite label
	ui.label_state->setText(u8"¶ªÊ§");
	ui.label_state->setStyleSheet("QLabel{color:#ab0000;background:transparent;}");

	//show demo
	this->widgetDemoShow();
}

widget_Demo::~widget_Demo()
{
	onStopThread();

	if (m_Trackerthread->isRunning())
	{
		m_Trackerthread->setThreadStatus(0);
		m_Trackerthread->quit();
		m_Trackerthread->wait();
	}

	if (m_Monitoringthread->isRunning())
	{
		m_Monitoringthread->setThreadStatus(0);
		m_Monitoringthread->quit();
		m_Monitoringthread->wait();
	}
}

void widget_Demo::OnUpdateTracking(std::vector<ToolTrackingData> trackingData)
{
	m_allMarker = trackingData;
	
	//show tracking data
	if (trackingData.size() > 0)
	{
		if (trackingData[0].matchStatus)
		{

			ui.label_state->setText(u8"¸ú×Ù");

			if (trackingData[0].transform.status == TransformationStatus::Enabled)
				ui.label_state->setStyleSheet("QLabel{color:#00b300;background:transparent;}");
			else
				ui.label_state->setStyleSheet("QLabel{color:#caca00;background:transparent;}");

			ui.label_mplane->setText(QString::number(trackingData[0].matchedPlane, 10));
			ui.label_qw->setText(QString::number(trackingData[0].transform.qw, 'f', 3));
			ui.label_qx->setText(QString::number(trackingData[0].transform.qx, 'f', 3));
			ui.label_qy->setText(QString::number(trackingData[0].transform.qy, 'f', 3));
			ui.label_qz->setText(QString::number(trackingData[0].transform.qz, 'f', 3));

			double matr[4][4];
			for (int j = 0; j < 4; j++)
			{
				for (int k = 0; k < 4; k++)
					matr[j][k] = trackingData[0].transform.matrix[j][k];
			}

			ui.label_alpha->setText(QString::number(trackingData[0].transform.roll, 'f', 3));
			ui.label_beta->setText(QString::number(trackingData[0].transform.pitch, 'f', 3));
			ui.label_gama->setText(QString::number(trackingData[0].transform.yaw, 'f', 3));

			if (trackingData[0].plane.markers[0].P[0] == -10000.0 || trackingData[0].plane.markers[0].P[1] == -10000.0 || trackingData[0].plane.markers[0].P[2] == -10000.0)
			{
				ui.label_1x->setText("/");
				ui.label_1y->setText("/");
				ui.label_1z->setText("/");
			}
			else
			{
				ui.label_1x->setText(QString::number(trackingData[0].plane.markers[0].P[0], 'f', 3));
				ui.label_1y->setText(QString::number(trackingData[0].plane.markers[0].P[1], 'f', 3));
				ui.label_1z->setText(QString::number(trackingData[0].plane.markers[0].P[2], 'f', 3));
			}

			if (trackingData[0].plane.markers[1].P[0] == -10000.0 || trackingData[0].plane.markers[1].P[1] == -10000.0 || trackingData[0].plane.markers[1].P[2] == -10000.0)
			{
				ui.label_2x->setText("/");
				ui.label_2y->setText("/");
				ui.label_2z->setText("/");
			}
			else
			{
				ui.label_2x->setText(QString::number(trackingData[0].plane.markers[1].P[0], 'f', 3));
				ui.label_2y->setText(QString::number(trackingData[0].plane.markers[1].P[1], 'f', 3));
				ui.label_2z->setText(QString::number(trackingData[0].plane.markers[1].P[2], 'f', 3));
			}

			if (trackingData[0].plane.markers[2].P[0] == -10000.0 || trackingData[0].plane.markers[2].P[1] == -10000.0 || trackingData[0].plane.markers[2].P[2] == -10000.0)
			{
				ui.label_3x->setText("/");
				ui.label_3y->setText("/");
				ui.label_3z->setText("/");
			}
			else
			{
				ui.label_3x->setText(QString::number(trackingData[0].plane.markers[2].P[0], 'f', 3));
				ui.label_3y->setText(QString::number(trackingData[0].plane.markers[2].P[1], 'f', 3));
				ui.label_3z->setText(QString::number(trackingData[0].plane.markers[2].P[2], 'f', 3));
			}

			if (trackingData[0].plane.markers[3].P[0] == -10000.0 || trackingData[0].plane.markers[3].P[1] == -10000.0 || trackingData[0].plane.markers[3].P[2] == -10000.0)
			{
				ui.label_4x->setText("/");
				ui.label_4y->setText("/");
				ui.label_4z->setText("/");
			}
			else
			{
				ui.label_4x->setText(QString::number(trackingData[0].plane.markers[3].P[0], 'f', 3));
				ui.label_4y->setText(QString::number(trackingData[0].plane.markers[3].P[1], 'f', 3));
				ui.label_4z->setText(QString::number(trackingData[0].plane.markers[3].P[2], 'f', 3));
			}

			if (trackingData[0].points.size() > 0)
			{
				ui.label_1vx->setText(QString::number(trackingData[0].points[0].P[0], 'f', 3));
				ui.label_1vy->setText(QString::number(trackingData[0].points[0].P[1], 'f', 3));
				ui.label_1vz->setText(QString::number(trackingData[0].points[0].P[2], 'f', 3));
			}
			
			ui.label_dx->setText(QString::number(trackingData[0].dir[0], 'f', 3));
			ui.label_dy->setText(QString::number(trackingData[0].dir[1], 'f', 3));
			ui.label_dz->setText(QString::number(trackingData[0].dir[2], 'f', 3));

			ui.label_tipx->setText(QString::number(trackingData[0].transform.tx, 'f', 3));
			ui.label_tipy->setText(QString::number(trackingData[0].transform.ty, 'f', 3));
			ui.label_tipz->setText(QString::number(trackingData[0].transform.tz, 'f', 3));
		}
		else
		{

			ui.label_state->setText(u8"¶ªÊ§");
			ui.label_state->setStyleSheet("QLabel{color:#ab0000;background:transparent;}");
			ui.label_qw->setText("/");
			ui.label_qx->setText("/");
			ui.label_qy->setText("/");
			ui.label_qz->setText("/");

			ui.label_1x->setText("/");
			ui.label_1y->setText("/");
			ui.label_1z->setText("/");

			ui.label_2x->setText("/");
			ui.label_2y->setText("/");
			ui.label_2z->setText("/");

			ui.label_3x->setText("/");
			ui.label_3y->setText("/");
			ui.label_3z->setText("/");

			ui.label_4x->setText("/");
			ui.label_4y->setText("/");
			ui.label_4z->setText("/");

			ui.label_1vx->setText("/");
			ui.label_1vy->setText("/");
			ui.label_1vz->setText("/");

			ui.label_dx->setText("/");
			ui.label_dy->setText("/");
			ui.label_dz->setText("/");

			ui.label_tipx->setText("/");
			ui.label_tipy->setText("/");
			ui.label_tipz->setText("/");

			ui.label_alpha->setText("/");
			ui.label_beta->setText("/");
			ui.label_gama->setText("/");

			ui.label_mplane->setText("/");
		}
	}
	else
	{
		ui.label_state->setText(u8"¶ªÊ§");
		ui.label_state->setStyleSheet("QLabel{color:#ab0000;background:transparent;}");
		ui.label_qw->setText("/");
		ui.label_qx->setText("/");
		ui.label_qy->setText("/");
		ui.label_qz->setText("/");

		ui.label_1x->setText("/");
		ui.label_1y->setText("/");
		ui.label_1z->setText("/");

		ui.label_2x->setText("/");
		ui.label_2y->setText("/");
		ui.label_2z->setText("/");

		ui.label_3x->setText("/");
		ui.label_3y->setText("/");
		ui.label_3z->setText("/");

		ui.label_4x->setText("/");
		ui.label_4y->setText("/");
		ui.label_4z->setText("/");

		ui.label_tipx->setText("/");
		ui.label_tipy->setText("/");
		ui.label_tipz->setText("/");

		ui.label_alpha->setText("/");
		ui.label_beta->setText("/");
		ui.label_gama->setText("/");

		ui.label_mplane->setText("/");
	}
}

void widget_Demo::OnUpdateUnMatchedMarkers(std::vector<MarkerPosition> markerData)
{
	//show unmatched markers
	ui.label_unMatchNum->setText(QString::number(markerData.size()));
}

void widget_Demo::OnUpdateAllMarkers(std::vector<MarkerPosition> markerData)
{
	//show all markers
	int Rnum = 0;
	int Fnum = 0;

	for (int i = 0; i < markerData.size(); i++)
	{
		if (markerData.at(i).type == 1)
			Rnum++;
		else if (markerData.at(i).type == 2)
			Fnum++;
	}

	ui.label_allMarkerNum->setText(QString::number(Rnum) + "-" + QString::number(Fnum));

	m_allMarkerData = markerData;

	OnUpdateTracking(m_Trackerthread->getTrackingData(markerData));
	
	OnUpdateUnMatchedMarkers(m_Trackerthread->getUnMatchedMarkers());
}

void widget_Demo::OnPtCloudBtn()
{
	m_Trackerthread->reconstructPointCloud(360, 363, 776, 418, 776, 418);
}

void widget_Demo::OnLuminanceChange(int value)
{
	ui.label_luminanceValue->setText(QString::number(value));
	double l = value / 100.0;
	m_Trackerthread->setLuminance(l);
}

void widget_Demo::OnContrastChange(int value)
{
	ui.label_contrastValue->setText(QString::number(value));
	double c = value / 100.0;
	m_Trackerthread->setContrast(c);
}

void widget_Demo::OnCapturePhotograph()
{
	//show imaging data
	QPixmap pixMapL = ui.label->pixmap(Qt::ReturnByValue);
	ui.label_5->setPixmap(pixMapL);
	ui.label_5->setScaledContents(true);
	ui.label_5->setAlignment(Qt::AlignCenter);
	
	QPixmap pixMapR = ui.label_2->pixmap(Qt::ReturnByValue);
	ui.label_6->setPixmap(pixMapR);
	ui.label_6->setScaledContents(true);
	ui.label_6->setAlignment(Qt::AlignCenter);

	//show monitoring data
	QPixmap pixMapC = ui.label_3->pixmap(Qt::ReturnByValue);
	ui.label_13->setPixmap(pixMapC);
	ui.label_13->setScaledContents(true);
	ui.label_13->setAlignment(Qt::AlignCenter);
	
	//pixMapL.save("./Loutput.png", "PNG");
	//pixMapR.save("./Routput.png", "PNG");
	//pixMapC.save("./Coutput.png", "PNG");
}

void widget_Demo::OnUpdateImaging(char* leftImage, char* rightImage, int cols, int rows)
{
	if (leftImage != nullptr)
	{
		QImage leftimg((const uchar*)leftImage, cols, rows, cols, QImage::Format_Indexed8);
		ui.label->setPixmap(QPixmap::fromImage(leftimg));
		ui.label->setScaledContents(true);
		ui.label->setAlignment(Qt::AlignCenter);
	}
	if (rightImage != nullptr)
	{
		QImage rightimg((const uchar*)rightImage, cols, rows, cols, QImage::Format_Indexed8);
		ui.label_2->setPixmap(QPixmap::fromImage(rightimg));
		ui.label_2->setScaledContents(true);
		ui.label_2->setAlignment(Qt::AlignCenter);
	}
}

void widget_Demo::OnUpdateMonitoring(unsigned char* image, int cols, int rows)
{
	//show imaging data
	if (image != nullptr)
	{
		QImage cneterimg((const uchar*)image, cols, rows, cols * 3, QImage::Format_RGB888);
		ui.label_3->setPixmap(QPixmap::fromImage(cneterimg));
		ui.label_3->setScaledContents(true);
		ui.label_3->setAlignment(Qt::AlignCenter);
	}
}

void widget_Demo::onStopThread()
{
	//stop thread
	m_Trackerthread->setThreadStatus(0);
	if (m_isMonitor)
		m_Monitoringthread->setThreadStatus(0);
}

void widget_Demo::onStartThread()
{
	//start thread
	m_Trackerthread->setThreadStatus(1);
	m_Trackerthread->start();

	if (m_isMonitor)
	{
		m_Monitoringthread->setThreadStatus(1);
		m_Monitoringthread->start();
	}
}

void widget_Demo::generateAROMfile()
{
	std::vector<MarkerPosition> markers;
	MarkerPosition a;
	a.P[0] = 0.00;
	a.P[1] = 0.00;
	a.P[2] = 0.00;
	a.P[3] = 1;
	MarkerPosition b;
	b.P[0] = 0.00;
	b.P[1] = -50.00;
	b.P[2] = 0.00;
	b.P[3] = 1;
	MarkerPosition c;
	c.P[0] = -25.00;
	c.P[1] = -100.00;
	c.P[2] = 0.00;
	c.P[3] = 1;
	MarkerPosition d;
	d.P[0] = 25.00;
	d.P[1] = -135.00;
	d.P[2] = 0.00;
	d.P[3] = 1;
	markers.push_back(a);
	markers.push_back(b);
	markers.push_back(c);
	markers.push_back(d);

	MarkerPosition point;
	point.P[0] = 0.00;
	point.P[1] = 0.00;
	point.P[2] = 0.00;
	point.P[3] = 1;

	MarkerPlane plane;
	plane.markers = markers;

	ToolCalibrationData newTool;
	newTool.name = "Tool";
	newTool.markerType = 0;
	newTool.planeNum = 1;
	newTool.minNumMarker = 3;
	newTool.calbError = -1;//no calibrated tip, the tip will be set to (0,0,0)
	newTool.planes.push_back(plane);
	newTool.points.push_back(point);
	newTool.dir[0] = 0;
	newTool.dir[1] = 0;
	newTool.dir[2] = 1;
	newTool.algorithmType = 0;
	//newTool.pin[0] = -0.08;
	//newTool.pin[1] = 158.26;
	//newTool.pin[2] = -18.21;

	//generate tool
	m_Trackerthread->generateAROM("./tool/", newTool);
}

void widget_Demo::configureTools()
{
	m_Trackerthread->loadPassiveToolAROM("./tool");
}

void widget_Demo::widgetDemoShow()
{
	//generate a testing standard arom file
	//this->generateAROMfile();

	//automatically or manually connect RT device
	//string hostname = "RT-MAX000001.local";
	//if (m_Trackerthread->connect(hostname) != 0)
	if (m_Trackerthread->autoConnect() != 0)
	{
		qDebug() << "Connection failed!";
		qDebug() << "Press Enter to continue...";
		cin.ignore();
		return;
	}
	else
		m_Trackerthread->printAPIversion();

	//get device type
	int DeviceType = m_Tracker->getConnectedDeviceInfo().deviceType;
	if (DeviceType == DeviceType::MAXV || DeviceType == DeviceType::PRO)
	{
		//creat monitoringWorker
		m_Monitoringthread = new monitoringWorker(m_Tracker);
		QObject::connect(m_Monitoringthread, SIGNAL(SendMonitoringInformation(unsigned char*, int, int)), this, SLOT(OnUpdateMonitoring(unsigned char*, int, int)));
		m_isMonitor = true;
		m_Trackerthread->startMonitoring();

		//reconstruction is only for PRO
		if (DeviceType == DeviceType::PRO)
			ui.ptCloudBtn->show();
	}
	else
	{
		ui.ptCloudBtn->hide();
	}

	//load all .arom files from the input path
	this->configureTools();

	//start tracking
	m_Trackerthread->startTracking();

	
	//start imaging
	m_Trackerthread->setAreaDisplay(360, 363, 776, 418, 776, 418);
	m_Trackerthread->startImaging();

	//start thread
	this->onStartThread();
}