#ifndef TOOLREGI_HPP
#define TOOLREGI_HPP

#ifdef ARMDCOMBINEDAPI_EXPORTS
#define ARMDCOMBINED_API __declspec(dllexport)
#else
#define ARMDCOMBINED_API __declspec(dllimport)
#endif

#include "ARMDAlgorithm.h"

#include <string>
#include <map>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>

namespace TransformationStatus
{
	enum values
	{
		Enabled = 0x0000,
		PartiallyOutOfVolume = 0x0001,
		OutOfVolume = 0x0002,
		PartiallyMatched = 0x0003,
		TooFewMarkers = 0x0004,
		BadTransformFit = 0x0005,
		TooLargeAngle = 0x0006,
		// = 0x0007,
		// = 0x0008,
		ToolMissing = 0x0009,
	};

	//converts the error code to std::string representation
	ARMDCOMBINED_API std::string toString(uint16_t errorCode);
}

struct ToolPlaneMarker
{
	std::vector<std::vector<int>> regiSet;
	std::vector<std::vector<int>> regiDifSet;
};

struct MarkerPosition
{
	double P[4] = { 0.0, 0.0, 0.0, 1 };//marker position (x,y,z,1)
	double dir[3] = { 1.0, 0.0, 0.0 };//marker direction (dx,dy,dz)
	int leftExposure = 0;//marker exposure
	int rightExposure = 0;//marker exposure
	int type = 0;//0(unknown), 1(passive), 2(active)
	bool RF = true;//true(real), false(phantom)
	double biase = -1;//biase after matching

	MarkerPosition operator*(double matrix[4][4]);
};

struct MarkerPlane
{
	int index = 0;
	std::vector<MarkerPosition> markers;//markers in the plane
	double dir[3] = { 0.0, 0.0, 0.0 };//plane direction (dx,dy,dz)
};

struct Transformation
{
	uint16_t status = TransformationStatus::ToolMissing;//transformation status
	double matrix[4][4];//transformation matrix [R|t]
	double qw, qx, qy, qz;//transformation quaternion
	double tx, ty, tz;//tip position
	double yaw, pitch, roll;//rotation angle

	Transformation inversed();//inverse transformation matrix [R|t]
	Transformation transformed();//transform information(pre-multiplied by the input matrix)
};

struct ToolTrackingData
{
	std::string name = "";//tool name
	std::string timespec = "";//acquisition time
	Transformation transform;//transform information
	int matchStatus = 0;//matching status
	int matchedPlane = 0;//matched plane index
	int matchedMarkersNum = 0;//matched markers number on the plane
	MarkerPlane plane;//positions of all markers on the matched plane
	std::vector<MarkerPosition> points;//positions of all virtual points
	double dir[3] = { 0.0, 0.0, 0.0 };//direction
	double error;//matching error
	double fps = 0;//matching fps

	ToolTrackingData projected();//project tracking data of the tool(source) to that of reference(target)
	ToolTrackingData filtered();//filter tracking data of the tool(source)
};

class ToolRegi
{
public:
	ToolRegi();
	~ToolRegi();

	//set device type
	void setDeviceType(int version);

	//input the tool to be tracked
	void SetInputData(std::string toolname, std::vector<std::vector<MarkerPosition>> source, std::vector<MarkerPosition> target, std::vector<ToolPlaneMarker> planeMarker,
		double tip[3], std::vector<MarkerPosition> point, double dir[3], std::vector<std::vector<double>> planeDir, int minMatchingNum, double maxMatchingAngle, double maxFRE, int algorithm, double matrix[4][4], std::string refToolName);
	
	void SetInputData2(std::string toolname, std::vector<MarkerPosition> source, std::vector<MarkerPosition> target, std::vector<ToolPlaneMarker> planeMarker, std::map<int, std::vector<int>> neighbor,
		double tip[3], std::vector<MarkerPosition> point, double dir[3], int minMatchingNum, double maxMatchingFRE, double matrix[4][4], std::string refToolName);

	//match markers to tool
	bool Register();
	bool Register2();
	bool Register3();
	bool RegisterR(ToolTrackingData& trackingDataOut, std::vector<MarkerPosition> targetMarkerIn, std::map<double, std::vector<MarkerPosition>>& twoPoints);
	void pointRegister(std::vector<MarkerPosition> source, std::vector<MarkerPosition> target, double Matrix[4][4]);
	
	//get tool tracking data
	void GetTrackingData(ToolTrackingData& data);
	static Transformation TransformedTrackingData(Transformation data);

	//project source tracking data to reference
	static ToolTrackingData projection(ToolTrackingData sourceTool);
	static ToolTrackingData filter(ToolTrackingData sourceTool);
	void setFilterParameter(std::string method, int param);

	//inverse tool tracking data
	static bool inverMatrix(double Matrix[4][4], double invMatrix[4][4]);

	//convert matrix to quaternion
	static void mat2Quat(Transformation& tran);

	//convert quaternion to matrix 
	static void quat2Mat(double Matrix[4][4], double w, double x, double y, double z);

	//marker in/out the tracking volume
	int judgePtInOutVolume(int device, std::vector<MarkerPosition> Pts);

protected:
	ARMDAlgorithm* AG;

	int deviceVersion = 0;
	std::vector<std::vector<MarkerPosition>> extent;

	ToolTrackingData trackingData;
	thread_local static ToolTrackingData refTrackingData;
	thread_local static std::map< std::string, std::vector<ToolTrackingData>> lastValuesList;
	thread_local static std::map< std::string, std::vector<double>> kalmanPmatrix;
	thread_local static std::map< std::string, std::vector<double>> kalmanXmatrix;

	std::vector<std::vector<MarkerPosition>> toolMarker;
	std::vector<MarkerPosition> targetMarker;
	std::vector<MarkerPosition> pointPosition;
	std::vector<MarkerPosition> targetRealMarker;
	std::vector<MarkerPosition> targetPhantomMarker;
	std::vector<ToolPlaneMarker> toolPlaneMarker;
	std::vector<std::vector<double>> toolPlaneDir;
	std::map<int, std::vector<int>> neighbor;
	std::vector<int> order;
	std::string referenceToolName;

	thread_local static double transMatrix[4][4];
	thread_local static double tipPoint[3];
	thread_local static double dir[3];
	double maxFRE = 1;
	int minRegNum = 3;
	double maxRegAngle = 60;
	int algorithmType = 1;

	thread_local static int filterNumber;
	thread_local static int filterMethod;

	//matching function
	std::vector<MarkerPosition> arrangeTri(std::vector<MarkerPosition> src, bool record);
	bool triMatch(std::vector<MarkerPosition> a, std::vector<MarkerPosition> b);
	int jacobiN(double** a, int n, double* w, double** v);
	void perpendiculars(const double v1[3], double v2[3], double v3[3], double theta);
	static double eucDist(double* a, double* b);

	//post-processing of matching result 
	std::vector<MarkerPosition> reOrderMarkers(std::vector<MarkerPosition>, std::vector<int> set, std::vector<int> difset, int planeIndex);
	std::vector<MarkerPosition> reOrderMarkers(std::vector<MarkerPosition>, std::vector<int> set);
};
#endif