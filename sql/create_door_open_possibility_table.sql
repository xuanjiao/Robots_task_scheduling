create database if not exists sensor_db character set utf8 collate utf8_general_ci;
use sensor_db;

drop table if exists open_possibility_table;
create table open_possibility_table(
		raw_id int,
        room_id char,
        day_of_week int,
        start_time time,
        end_time time,
        open_posibility int,
        statistic_door_open_posibility int
    );

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'00:00:00','9:59:59',0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'10:00:00','11:59:59',90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'12:00:00','12:59:59',10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'13:00:00','14:59:59',90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'a',2,'15:00:00','23:59:59',0);


insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',2,'00:00:00','11:59:59',0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',2,'12:00:00','17:59:59',90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'b',2,'18:00:00','23:59:59',0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',2,'00:00:00','7:59:59',0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',2,'8:00:00','10:59:59',90);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',2,'11:00:00','11:59:59',10);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'c',2,'12:00:00','23:59:59',0);

insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'd',2,'00:00:00','7:59:59',0);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'd',2,'8:00:00','12:59:59',100);
insert into open_possibility_table(room_id,day_of_week,start_time,end_time,open_posibility) values( 'd',2,'13:00:00','23:59:59',0);


SET @row=0;
UPDATE open_possibility_table SET raw_id=(@row:=@row+1);
            
select * from open_possibility_table