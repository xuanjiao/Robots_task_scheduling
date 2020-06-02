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
drop procedure if exists process_measurement_result;

delimiter $$
create procedure process_measurement_result(room char,t datetime )
begin
    declare pos int default 0; 
	declare time_left time default 0;
    declare time_right time default 0;
    declare d_w int default 0;
    
    set d_w = dayofweek(t);
    -- find time slot
	select start_time,end_time into time_left,time_right
			from user_defined_door_open_possibility_table
    where room = room_id and d_w = day_of_week and (time(t) between start_time and end_time) ;
    
     -- select time_left,time_right,d_w; 

   select 100*sum(door_status)/count(door_status) into pos -- calculate possibility
	
                     from door_status_list
		where room = room_id and d_w = dayofweek(date_time) and (TIME(date_time) between time_left and time_right);
	
   -- select pos,time_left,time_right;

	SET SQL_SAFE_UPDATES = 0;	
	update user_defined_door_open_possibility_table  -- update possibility table with new posibility
		set statistic_door_open_posibility = pos
    where room = room_id and time_left = start_time and time_right = end_time and d_w = day_of_week;
    
end $$

delimiter ;

-- declare door_open_pos int  default 0;

--  call update_possibility_table();
 