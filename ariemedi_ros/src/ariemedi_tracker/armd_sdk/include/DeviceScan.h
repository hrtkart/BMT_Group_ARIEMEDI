#ifndef DEVICESCAN_H
#define DEVICESCAN_H

#ifdef ARMDCOMBINEDAPI_EXPORTS
#define ARMDCOMBINED_API __declspec(dllexport)
#else
#define ARMDCOMBINED_API __declspec(dllimport)
#endif

#include "UdpSearchInfo.h"

#include <map>
#include <string>
#include <iostream>
#include <stdlib.h>

class DeviceScan
{
public:
	explicit DeviceScan();
	~DeviceScan();

	/**
	* @brief: update reachable device hostname and IP 
	*/
	void updateDeviceInfo();

	/**
	* @brief: get reachable device hostname and IP  
	* @return: map of hostname and IPs (key: device's hostname, value:{device's IP, adaptor's IP, adaptor's mask, reachable})
	*/
	std::map<std::string, std::vector<std::string>> getDeviceInfo();

private:
	UdpSearchInfo* UdpSearcher;

	std::map<std::string, std::vector<std::string>> mapHostIP;
};
#endif
