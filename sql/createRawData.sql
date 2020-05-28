create database if not exists sensor_db character set utf8 collate utf8_general_ci;
use sensor_db;
drop table if exists door_status_list;
create table door_status_list(
    room_id char,
	door_status boolean,
    date_time datetime
    );

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
            if(rand()*100 < open_percent,1,0), 
            cur_time -- create door status according to possibility table
                from user_defined_door_open_possibility_table
		    where TIME(cur_time) >= start_time and TIME(cur_time)< end_time and id = room_id;

          set cur_time = addtime(cur_time,period);
          set cnt = cnt + 1;
    end while;

    end $$
delimiter ;

-- create raw data with datetime and door status, 
call createRawData('a','2020-06-01 8:00:00','00:10:00',100);
call createRawData('b','2020-06-01 8:00:00','00:10:00',100);
call createRawData('c','2020-06-01 8:00:00','00:10:00',100);
call createRawData('d','2020-06-01 8:00:00','00:10:00',100);

select * from door_status_list;
