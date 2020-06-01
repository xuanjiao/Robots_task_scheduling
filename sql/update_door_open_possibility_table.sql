use sensor_db;

drop function if exists get_door_open_possibility;
delimiter ;;
create function get_door_open_possibility(room char, day_of_week int, start_time time, end_time time) 
returns int DETERMINISTIC
begin
    declare open_percent int  default 0; 
    -- count door open times in raw data file and calculate possibility 
    select 100*sum(door_status)/count(door_status) into open_percent
                from door_status_list
                where room = room_id and day_of_week = dayofweek(date_time) and TIME(date_time) between start_time and end_time;
   
    return open_percent;
end ;;
delimiter ;
-- get a line from door open possibility table 
drop procedure if exists update_possibility_table;

delimiter $$
create procedure update_possibility_table()
begin
    declare n int default 0;
    declare i int default 0;
    declare open_percent int  default 0; 


    select count(*) from  user_defined_door_open_possibility_table into n; -- get num of rows
	
    update user_defined_door_open_possibility_table  -- calculate new posibility table according to raw data
           set statistic_door_open_posibility = get_door_open_possibility(room_id,day_of_week,start_time,end_time);

end $$

delimiter ;

-- declare door_open_pos int  default 0;

--  call update_possibility_table();
 