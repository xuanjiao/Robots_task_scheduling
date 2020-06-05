create database if not exists sensor_db character set utf8 collate utf8_general_ci;
use sensor_db;

drop table if exists open_possibility_table;
create table open_possibility_table(
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

-- Monday
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',2,'00:00:00','9:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',2,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',2,'11:00:00','11:59:59',80,80);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',2,'12:00:00','12:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',2,'13:00:00','13:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',2,'14:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',2,'00:00:00','11:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',2,'12:00:00','12:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',2,'13:00:00','13:59:59',100,100);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',2,'14:00:00','14:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',2,'15:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',2,'00:00:00','7:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',2,'8:00:00','8:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',2,'9:00:00','9:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',2,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',2,'11:00:00','23:59:59',0,0);


-- Tuesday
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',3,'00:00:00','9:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',3,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',3,'11:00:00','11:59:59',80,80);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',3,'12:00:00','12:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',3,'13:00:00','13:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',3,'14:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',3,'00:00:00','11:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',3,'12:00:00','12:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',3,'13:00:00','13:59:59',100,100);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',3,'14:00:00','14:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',3,'15:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',3,'00:00:00','7:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',3,'8:00:00','8:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',3,'9:00:00','9:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',3,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',3,'11:00:00','23:59:59',0,0);

-- Wednesday
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',4,'00:00:00','9:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',4,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',4,'11:00:00','11:59:59',80,80);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',4,'12:00:00','12:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',4,'13:00:00','13:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',4,'14:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',4,'00:00:00','11:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',4,'12:00:00','12:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',4,'13:00:00','13:59:59',100,100);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',4,'14:00:00','14:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',4,'15:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',4,'00:00:00','7:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',4,'8:00:00','8:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',4,'9:00:00','9:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',4,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',4,'11:00:00','23:59:59',0,0);


-- Tursday
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',5,'00:00:00','9:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',5,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',5,'11:00:00','11:59:59',80,80);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',5,'12:00:00','12:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',5,'13:00:00','13:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',5,'14:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',5,'00:00:00','11:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',5,'12:00:00','12:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',5,'13:00:00','13:59:59',100,100);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',5,'14:00:00','14:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',5,'15:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',5,'00:00:00','7:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',5,'8:00:00','8:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',5,'9:00:00','9:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',5,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',5,'11:00:00','23:59:59',0,0);

-- Friday
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',6,'00:00:00','9:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',6,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',6,'11:00:00','11:59:59',80,80);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',6,'12:00:00','12:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',6,'13:00:00','13:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'a',6,'14:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',6,'00:00:00','11:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',6,'12:00:00','12:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',6,'13:00:00','13:59:59',100,100);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',6,'14:00:00','14:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'b',6,'15:00:00','23:59:59',0,0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',6,'00:00:00','7:59:59',0,0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',6,'8:00:00','8:59:59',10,10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',6,'9:00:00','9:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',6,'10:00:00','10:59:59',90,90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility,statistic_door_open_posibility) values( 'c',6,'11:00:00','23:59:59',0,0);


-- 
-- -- -- create raw data with datetime and door status, 
call createRawData('a','2020-06-01 0:00:00','00:10:00',500);
call createRawData('b','2020-06-01 0:00:00','00:10:00',500);
call createRawData('c','2020-06-01 0:00:00','00:10:00',500);

select * from door_status_list;
select * from open_possibility_table;