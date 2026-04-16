#ifndef SYSTEM_CRC_TEST_HPP
#define SYSTEM_CRC_TEST_HPP

/**
 * @brief This class contains methods and data used to verify Cyclical Redundancy Checks (CRCs)
 */

class SystemCRCTest
{
public:
	SystemCRCTest();
	virtual ~SystemCRCTest() {};

	//cyclic redundancy check
	unsigned int calculateCRC16Test(const char* reply, int replyLength) const;

private:
	unsigned int calcValue(unsigned int crc, int data) const;

	static unsigned int crcTable_[256];
};

#endif // SYSTEM_CRC_TEST_HPP