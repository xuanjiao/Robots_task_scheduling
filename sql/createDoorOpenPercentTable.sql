create database if not exists sensor_db character set utf8 collate utf8_general_ci;
use sensor_db;

drop table if exists user_defined_door_open_possibility_table;
create table user_defined_door_open_possibility_table(
        room_id char,
        day_of_week int,
        start_time time,
        end_time time,
        open_percent float
    );

insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'a',1,'00:00:00','10:00:00',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'a',1,'10:00:00','15:00:00',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'a',1,'15:00:00','18:00:00',50);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'a',1,'18:00:00','24:00:00',0);

insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'b',1,'00:00:00','12:00:00',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'b',1,'12:00:00','18:00:00',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'b',1,'18:00:00','24:00:00',0);

insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'c',1,'00:00:00','8:00:00',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'c',1,'8:00:00','12:00:00',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'c',1,'8:00:00','12:00:00',90);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'c',1,'12:00:00','24:00:00',0);

insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'd',1,'00:00:00','8:00:00',0);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'd',1,'8:00:00','15:00:00',100);
insert into user_defined_door_open_possibility_table(room_id,day_of_week,start_time,end_time,open_percent) values( 'd',1,'15:00:00','24:00:00',0);

select * from user_defined_door_open_possibility_table