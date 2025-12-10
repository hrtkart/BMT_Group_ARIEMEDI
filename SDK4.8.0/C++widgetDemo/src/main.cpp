#include "widget_Demo.h"
#include <QtWidgets/QApplication>
#pragma comment(linker, "/subsystem:\"console\" /entry:\"mainCRTStartup\"" ) 

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    widget_Demo widgetDemo;
    widgetDemo.show();
    return app.exec();
}
