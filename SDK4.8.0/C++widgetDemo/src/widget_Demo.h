#pragma once

#include "ui_widget_Demo.h"
#include "trackingWorker.h"
#include "monitoringWorker.h"
#include "ARMDCombinedAPI.h"

#include <QtWidgets/QMainWindow>
#include <QDebug>
#include <QPixmap.h>


using namespace std;

class widget_Demo : public QMainWindow
{
    Q_OBJECT

public:
    widget_Demo(QWidget *parent = nullptr);
    ~widget_Demo();

    //generate arom files
    void generateAROMfile();

    //load arom files
    void configureTools();

    //show widget demo
    void widgetDemoShow();

private slots:
    //receive tool tracking data
    void OnUpdateTracking(std::vector<ToolTrackingData> trackingData);

    //receive unmatched markers tracking data
    void OnUpdateUnMatchedMarkers(std::vector<MarkerPosition> markerData);

    //receive all markers tracking data
    void OnUpdateAllMarkers(std::vector<MarkerPosition> markerData);

    //receive imaging data
    void OnUpdateImaging(char* leftImage, char* rightImage, int cols, int rows);
    void OnUpdateMonitoring(unsigned char* image, int cols, int rows);
	void OnCapturePhotograph();
    void OnPtCloudBtn();
    void OnLuminanceChange(int value);
    void OnContrastChange(int value);

    //start and stop thread
    void onStopThread();
    void onStartThread();

private:
    Ui::widget_DemoClass ui;

    ARMDCombinedAPI* m_Tracker;

    trackingWorker* m_Trackerthread;
    monitoringWorker* m_Monitoringthread;

    bool m_isMonitor = false;

    std::vector<ToolTrackingData> m_allMarker;
    std::vector<MarkerPosition> m_allMarkerData;
};
