create database if not exists sensor_db character set utf8 collate utf8_general_ci;
use sensor_db;

drop table if exists user_defined_door_open_possibility_table;
create table user_defined_door_open_possibility_table(
        room_id char,
        day_of_week int,
        start_time time,
        end_time time,
        open_posibility int,
        statistic_door_open_posibility int
    );

drop table if exists door_status_list;
create table door_status_list(
    room_id char,
	door_status boolean,
    date_time datetime
    );

-- insert value to possibility table
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'00:00:00','9:59:59',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'10:00:00','11:59:59',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'12:00:00','12:59:59',10);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'13:00:00','14:59:59',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'15:00:00','23:59:59',0);

insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',2,'00:00:00','11:59:59',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',2,'12:00:00','17:59:59',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',2,'18:00:00','23:59:59',0);

insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',2,'00:00:00','7:59:59',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',2,'8:00:00','10:59:59',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',2,'11:00:00','11:59:59',10);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',2,'12:00:00','23:59:59',0);

-- Tuesday
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',3,'00:00:00','9:59:59',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',3,'10:00:00','11:59:59',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',3,'12:00:00','12:59:59',10);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',3,'13:00:00','14:59:59',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',3,'15:00:00','23:59:59',0);

insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',3,'00:00:00','11:59:59',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',3,'12:00:00','17:59:59',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',3,'18:00:00','23:59:59',0);

insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',3,'00:00:00','7:59:59',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',3,'8:00:00','10:59:59',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',3,'11:00:00','11:59:59',10);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',3,'12:00:00','23:59:59',0);


-- add raw number to door_open_possibility_table
-- SET @row=0;
-- UPDATE user_defined_door_open_possibility_table SET raw_id=(@row:=@row+1);      
--       
select * from user_defined_door_open_possibility_table;
-- 
-- -- create raw data with datetime and door status, 
call createRawData('a','2020-06-01 0:00:00','00:10:00',500);
call createRawData('b','2020-06-01 0:00:00','00:10:00',500);
call createRawData('c','2020-06-01 0:00:00','00:10:00',500);
-- call process_measurement_result('a','2020-06-01 10:00:00');

select * from door_status_list;
select * from user_defined_door_open_possibility_table;