#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP

#include <cmath>
#include <vector>

class ARMDAlgorithm
{
public:
	ARMDAlgorithm();
	virtual ~ARMDAlgorithm() {};

	static std::vector<double> getKalmanFilteredTrackingData(std::vector<double> currentData, std::vector<double>& Pmatrix, std::vector<double>& xmatrix, bool flag);

protected:
};
#endif // ALGORITHM_HPP
#pragma once