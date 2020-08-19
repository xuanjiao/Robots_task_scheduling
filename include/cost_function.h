#pragma once
#include "general_task.h"
struct TaskWeightBase{
    double W_BATTERY       = 10;
    double W_TIME          = 0.1;
    double W_POSSIBILITY   = 10;
    double W_PRIORITY      = 10;
}TWB;

struct DoorWeightBase{
    double W_TIME          = 0.1;
    double W_BATTERY       = 10;
    double W_POSSIBILITY   = 10;
}DWB;

class CostCalculator{
    public:
    static void CalculateLargeTaskCost(LargeTask& t){
        t.cost =  TWB.W_BATTERY/ t.tasks.size() * t.battery 
        + TWB.W_TIME  * t.waitingTime.toSec() 
        + TWB.W_POSSIBILITY * t.openPossibility 
        + TWB.W_PRIORITY * t.priority;
    }
    static double CalculateDoorCost(){
        return 0;
    }
};
// llcf = {10,10/ntaks,0.1,-10,-1)}