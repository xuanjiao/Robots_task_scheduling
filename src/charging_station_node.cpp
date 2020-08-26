#include "ros/ros.h"
#include "sql_client.h"

class ChargingStationSimulator{
public:
    ChargingStationSimulator(){

    }
    ChargingStationSimulator(SQLClient& sc):_sc(sc){}
    SQLClient _sc;
};

int main(){
    SQLClient sc("charging_station","pass");
    ChargingStationSimulator cs(sc);
}