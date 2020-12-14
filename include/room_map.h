#pragma once

#include "ros/ros.h"
using namespace std;

typedef pair<int,int> RoomPair;
typedef vector<int> Value;
typedef set<int> RltDoors;

class RoomMap{
public: 
  static map<RoomPair,RltDoors>  ROOMMAP;
  static RltDoors getRelativeDoors(int room1, int room2);
};
