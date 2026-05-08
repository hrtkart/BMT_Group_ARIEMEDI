#ifndef UDP_CONNECTION_HPP
#define UDP_CONNECTION_HPP

#include <string>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sstream>
#include <fstream>
#include <net/if.h>
#include <cstring>

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib, "iphlpapi.lib")

#include <stdint.h>
typedef uint8_t byte_t;

#define DEVICE_PORT 8080
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR,12)

class UdpConnection
{
public:
	UdpConnection();
	~UdpConnection();

	//initiation
	void setDeviceInfo(std::string hostname, int port, bool errorPrint);

	//read and write information from device
	int read(byte_t* buffer, int length) const;
	int write(byte_t* buffer, int length) const;

	//clean buffer
	void cleanBuff();

	//set receive time out
	void recTimeOut(int t);

	//get connection status
	int isConnected() const;

	//close UDP connection
	int closeUDP();

	//get connection IP
	std::vector<std::string> getConnectionIP();

	//get connection MAC
	std::vector<std::string> getConnectionMAC();

	//get connection mask
	std::string getConnectionMask();

	enum ConnectionError
	{
		nonError = 0,
		resloveError = -1,
		initialUDPError = -2,
		portConflicts = -3,
		invalidAddr = -4,
		deviceInexist = -5,
		timeOut = -6,
		notReachable = -7,
		unknownError = -10,
	};

private:
	int connectionInfo;
	
	int deviceSocket_;
	sockaddr_in addrSrv;
	
	std::string localmask;

	std::string deviceIP;
	std::string localIP;
	int portNumber;

	bool errorShow;

	//initialize UDP connection
	int init(std::string LocalIP, std::string DeviceIP);

	//get device and local IP
	std::string getDevicIP(std::string hostname);
	std::string getLocalIP(std::string DeviceIPv4);
	int getInfo(std::string text, std::vector<std::string>& Info);

	//get MAC of the input IP address
	std::string getMAC(std::string Ipv4);
};
#endif
#pragma once