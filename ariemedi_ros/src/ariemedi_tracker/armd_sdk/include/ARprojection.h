#ifndef ARPROJECTION_H
#define ARPROJECTION_H

#ifdef ARMDCOMBINEDAPI_EXPORTS
#define ARMDCOMBINED_API __declspec(dllexport)
#else
#define ARMDCOMBINED_API __declspec(dllimport)
#endif

#include "ReconstructPointCloud.h"

#include <opencv2/opencv.hpp>

class ARprojection
{
public:
	ARprojection();
	~ARprojection();

	/**
	* @brief: AR projection initialization
	* @param1: projection parameters
	* @param2: resolution, true-high resolution (default), false-low resolution
	* @return: succeed or failed
	*/
	bool initialization(std::vector<double> infor, bool r = true);

	/**
	* @brief: get AR projected point cloud in the coordinate of the monitor image
	* @param1: original point cloud to be projected
	* @param2: corresponding tracking data
	* @return: projection point cloud
	*/
	std::vector<cv::Point3d> getARprojection(std::vector<cv::Point3d> orgPointCloud, double trackingMatrix[4][4]);
	
private:
	cv::Mat projectionMatrix;
	cv::Mat remapMatrix;
	bool resolution;
};
#endif // ARPROJECTION_H
#pragma once
