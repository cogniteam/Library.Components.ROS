#ifndef ALGORITHMS_ANOMALYREPORT_H
#define ALGORITHMS_ANOMALYREPORT_H
#include "std_msgs/String.h"

using namespace std;

class AnomalyReport {

public:
    std_msgs::String description;
    float timeStep;

public:
    AnomalyReport(const std_msgs::String description, const float timeStep) : timeStep(0), description(description) {
        this->timeStep = timeStep;
    }
};

#endif //ALGORITHMS_ANOMALYREPORT_H