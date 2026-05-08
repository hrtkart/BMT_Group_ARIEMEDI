#ifndef RECONSTRUCTPOINTCLOUD_H
#define RECONSTRUCTPOINTCLOUD_H

#ifdef ARMDCOMBINEDAPI_EXPORTS
#define ARMDCOMBINED_API __declspec(dllexport)
#else
#define ARMDCOMBINED_API __declspec(dllimport)
#endif

#include <vector>
#include <iostream>
#include <fstream>
#include <omp.h>

#include <opencv2/opencv.hpp>

struct PointInfo
{
	float x, y, z;
	bool textured;
	unsigned char r, g, b, a;
	int p, q;
};

struct PointCloudInfo
{
	std::vector<PointInfo> points;
	int num = 0;
	int stage = -1;

	PointInfo getPoint(int index);
	PointCloudInfo getPoints(std::vector<int> index);
	PointCloudInfo getTexturedPoints();
	PointCloudInfo getUntexturedPoints();
	PointCloudInfo transformed(double matrix[4][4]);
	void removeUntexturedPoints();
	void insertPoint(PointInfo point);
	void insertPoints(std::vector<PointInfo> points);
	void concatenate(PointCloudInfo pc);
	std::vector<float> getBoundingBox();
	std::vector<float> getBoundCenter();
	std::vector<float> getGeometricalCenter();
};

class ReconstructPointCloud
{
public:
	ReconstructPointCloud();
	~ReconstructPointCloud();

	/**
	* @brief: initialize parameters for reconstruction
	* @param: parameters for reconstruction
	*/
	void initializeParameters(std::vector<double> infor);

	/**
	* @brief: input data for reconstruction
	* @param: data for reconstruction
	*/
	bool setReconstructData(std::vector<std::vector<char>> data);

	/**
	* @brief: set max displacement
	* @param: max displacement
	*/
	void setMaxDisp(int value);

	/**
	* @brief: get reconstruct pointcloud
	* @param: textured point cloud or non-textured point cloud
	* @return: pointcloud
	*/
	PointCloudInfo getPointCloud(bool texture);

	/**
	* @brief: get texture image with size of 1920*1200
	* @return: texture image with type of cv::Mat
	*/
	cv::Mat getTextureImageMat();
	
	/**
	* @brief: get texture image with size of 1920*1200
	* @return: texture image with type of unsigned char*
	*/
	unsigned char* getTextureImageUChar();

	/**
	* @brief: get depth image
	* @return: depth image with type of cv::Mat
	*/
	cv::Mat getDepthImage();

	/**
	* @brief: get depth image size
	* @return: depth image size (width, height)
	*/
	std::vector<int> getDepthImageSize();

	/**
	* @brief: project depth image to point cloud
	* @param1: depth image
	* @param2: textured point cloud or non-textured point cloud
	* @return: point cloud
	*/
	PointCloudInfo projectDepthImage2PointCloud(cv::Mat depthImage, bool texture);

	/**
	* @brief: set default color for the point without textured
	* @param1: red (0-255)
	* @param2: green (0-255)
	* @param3: blue (0-255)
	* @param4: alpha (0-255)
	*/
	void setDefaultColor(int r, int g, int b, int a);

	/**
	* @brief: project the input 2D texture image point to a 3D point in point cloud
	* @param1: image point-x
	* @param2: image point-y
	* @return: the corresponding index set of the points in cloud
	*/
	std::vector<int> project2DImagePosition(int x, int y);

	/**
	* @brief: project the input set of 2D image points to a 3D point are in point cloud
	* @param: set of image point
	* @return: the corresponding index set of the points in cloud
	*/
	std::vector<int> project2DImageArea(std::vector<std::pair<int, int>> area);

	/**
	* @brief: project the input rectangular area of 2D image points to a 3D point are in point cloud
	* @param1: center image point-x
	* @param2: center image point-y
	* @param3: width 
	* @param4: height
	* @return: the corresponding index set of the points in cloud
	*/
	std::vector<int> project2DImageRectArea(int cx, int cy, int w, int h);

	/**
	* @brief: project the input circle area of 2D image points to a 3D point are in point cloud
	* @param1: center image point-x
	* @param2: center image point-y
	* @param3: radius of the circle
	* @return: the corresponding index set of the points in cloud
	*/
	std::vector<int> project2DImageCircleArea(int cx, int cy, int r);

	/**
	* @brief: save reconstructed pointcloud
	* @param1: save path
	* @param2: pointcloud with the float type
	* @param3: w/o texture
	* @return: save success(true) or failure(false)
	*/
	bool savePLY(std::string filename, PointCloudInfo cloud, bool texture);

	/**
	* @brief: save reconstructed pointcloud
	* @param1: save path
	* @param2: pointcloud with the cv::Point3f type
	* @return: save success(true) or failure(false)
	*/
	bool savePLY(std::string filename, std::vector<cv::Point3f> cloud);

private:
	int deviceType = 1;
	cv::Mat
		KK_L, KK_R,
		KK_visual, R_visual, T_visual, D_visual,
		P_Visual, REC_LV_R, REC_LV_T, REC_L_K, L_To_rec_R,
		R, T, Dist_L, Dist_R;
	
	cv::Mat KV;
	cv::Mat recRv;
	cv::Mat TV3;

	cv::Mat rmap_L[2], rmap_R[2], remap_Visual[2];
	cv::Mat R_L, R_R, P_L, P_R;
	cv::Mat correctMat;
	cv::Mat CvcorrectH;
	cv::Mat convertMat;
	cv::Rect validROI_L, validROI_R;
	cv::Mat Q;
	//cv::Mat depthImage;
	cv::Size imageSize, imageVisSize, winSize;
	int wh;
	float pha_dif;
	double epiLength;

	int maxDisp;

	cv::Mat m_maskLeft;
	cv::Mat m_maskRight;
	std::vector<cv::Mat> m_imgL;
	std::vector<cv::Mat> m_imgR;
	cv::Mat m_imageV;

	std::vector<int> color;

	std::map<std::pair<int, int>, std::vector<int>> map2D3D;

	//reconstruct method
	bool rectify(cv::Mat& L, cv::Mat& R, cv::Mat& L_des, cv::Mat& R_des, cv::Mat& L_mask, cv::Mat& R_mask, cv::Rect& L_rect, cv::Rect& R_rect);
	bool init(int win_size, float pha_dif);
	int phaseSearch(cv::Mat& BOXA_L, cv::Mat& phaR, int Y_R, int X_R_Start);
	void calculatePhaseParallel(cv::Mat& pha_L, cv::Mat& pha_R, cv::Mat& B_L, cv::Mat& B_R);
	bool calcDistance(cv::Mat& dif_map, cv::Mat& depth_map, std::vector<std::pair<cv::Point2f, cv::Point2f>>& cps, cv::Mat Q);
	void filtNoise(cv::Mat& srcMat, cv::Mat disp_right = cv::Mat());
	void colorPoints(PointCloudInfo& points);

	//test
	//cv::Mat getDepthImage2();
	//bool setReconstructData(std::vector<std::vector<char>> data, bool color);
};
#endif // RECONSTRUCTPOINTCLOUD_H
