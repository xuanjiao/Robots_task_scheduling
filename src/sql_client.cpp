/* Standard C++ includes */
#include "sql_client.h"

int main(int argc, char**argv){
  ros::init(argc,argv,"sql_client");
  ros::Time::init();
  ros::Duration(1).sleep();
  ros::NodeHandle nh;
  SQLClient sql_client = SQLClient::getInstance();
  Table_row row;
  row.room_id = "a";
  sql_client.query_posibility_table_single_room(row,ros::Time::now());

  ros::spin();
}