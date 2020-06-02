
drop procedure if exists createRawData;

delimiter $$
create procedure createRawData(
    id char,
	start_date_time datetime,
    period time,
    cnt_total int
)

begin    	
	declare cnt int default 0;
    declare cur_time datetime;
    declare door boolean;
    set cur_time = start_date_time;
	while cnt < cnt_total do
          insert into door_status_list
          select 
            id,
            if(rand()*100 < open_posibility,1,0), 
            cur_time -- create door status according to possibility table
                from user_defined_door_open_possibility_table
		    where TIME(cur_time) between start_time and end_time and id = room_id and dayofweek(cur_time) = day_of_week;

			 call process_measurement_result(id,cur_time);
          set cur_time = addtime(cur_time,period);
          set cnt = cnt + 1;
    end while;

    end $$
delimiter ;

-- -- create raw data with datetime and door status, 
-- call createRawData('a','2020-06-01 0:00:00','00:10:00',200);
-- call createRawData('b','2020-06-01 0:00:00','00:10:00',200);
-- call createRawData('c','2020-06-01 0:00:00','00:10:00',200);
-- call createRawData('d','2020-06-01 0:00:00','00:10:00',200);

-- select * from door_status_list;
