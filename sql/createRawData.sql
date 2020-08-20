
drop procedure if exists createRawData;

delimiter $$
create procedure createRawData(
    in id int,
	in start_date_time datetime,
    in period time,
    in cnt_total int
)

begin    	
	declare cnt int default 0;
    declare cur_time datetime;
    set cur_time = start_date_time;
	while cnt < cnt_total do
          insert into measurements
          select 
            id,
            if(rand()< open_pos,1,0), 
            cur_time -- create door status according to possibility table
                from open_possibilities o
		    where TIME(cur_time) between o.start_time and o.end_time and id = o.door_id and dayofweek(cur_time) = o.day_of_week;
			 -- call process_measurement_result(id,cur_time);
          set cur_time = addtime(cur_time,period);
          set cnt = cnt + 1;
    end while;

    end $$
delimiter ;
