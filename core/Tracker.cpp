#include "Tracker.h"

Tracker::Tracker() : firstDate(true), alpha(0.2) {
    estimated_state = {0, 0, 0, 0};
}

void Tracker::ProcessData(double x, double y){
    if(firstDate){
        estimated_state.x = x;
        estimated_state.y = y;
        firstDate = false;
    }else{
        estimated_state.x = (1.00 - alpha) * estimated_state.x + alpha * x;
        estimated_state.y = (1.00 - alpha) * estimated_state.y + alpha * y;

    }
}

State Tracker::GetEstimatedState() const{
    return estimated_state;
}