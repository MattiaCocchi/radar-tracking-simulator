#ifndef TRACKER_H
#define TRACKER_H


#include <iostream>

#include "Entity.h"

class Tracker{
    public:
    Tracker();
    void ProcessData(double x, double y);
    State GetEstimatedState() const;
    private:
    State estimated_state;
    bool firstDate;
    double alpha;
};

#endif