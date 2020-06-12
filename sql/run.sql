create database if not exists sensor_db character set utf8 collate utf8_general_ci;
use sensor_db;

drop table if exists open_possibility_table;
create table open_possibility_table(
        room_id char(1),
        day_of_week int,
        start_time time,
        end_time time,
        open_posibility double(4,1),
        statistic_door_open_posibility double(4,1),
        constraint Room_Time primary key (day_of_week,start_time,end_time,room_id)
    );

drop table if exists door_status_list;
create table door_status_list(
    room_id char(1),
	door_status boolean,
    date_time datetime,
    constraint Room_Date_Time primary key(date_time,room_id)
    );

drop table if exists door_position;
create table door_position(
    room_id char(1),
    position_x double(14,12),
    position_y double(14,12),
    orientation_z double(14,12),
    orientation_w double(14,12),
    primary key (room_id)
);

drop table if exists charging_station_position;
create table charging_station_position(
    station_id char(1),
    position_x double(14,12),
    position_y double(14,12),
    orientation_z double(14,12),
    orientation_w double(14,12),
    primary key (station_id)
);


-- insert value to door_position table
-- insert door_position values('a',-21.6662127429, 9.50513223651, 9.50513223651, 0.667743723726);
-- insert door_position values('b',-19.4278817305, 9.0054257766, 0.673662651895, 0.739038991827);
-- insert door_position values('c',-14.7790163094, 9.0879504651, 0.696015604107, 0.718026656078);
-- insert door_position values('d',-22.3155041827, 8.20714003079, -0.999991890265,0.0040273322644);
-- insert door_position values('e',-17.6463588911, -17.6463588911, 0.710953141288, 0.703239383775);
-- insert door_position values('f',-13.2646228952, 7.77758415791, 7.77758415791,0.998654018142);
-- insert door_position values('g',-9.72013338947, 8.52865724571, -0.999999263125, 0.0012139806762);
-- insert door_position values('h',-9.03927424531, 6.62321407921, 0.651078250306, 0.759010613877);
-- insert door_position values('i',-0.868440756591, 7.0788002511, 0.662118779871, 0.749398906686);
-- insert door_position values('j',4.38077210276, 9.34650744461, 0.721488227349, 0.692426702112);
-- insert door_position values('k',6.48236977439, 9.48845367671, 0.711113291282,0.703077440231);
-- insert door_position values('l',3.6025773004, 8.71864545073, -0.99768098292, 0.0680636196487);
insert door_position values('a',4.23119675634, 7.38657063027, 0.608479240054, 0.793569791778);
insert door_position values('b',6.86290545247, 5.40794760884, 0.114759478636, 0.993393306834);
insert door_position values('c',-2.95220390532, 2.39211836475, -0.702331560869, -0.702331560869);
insert door_position values('d',4.28632121392, 2.54533955273, -0.723120814312, 0.690721570468);

-- insert value to possibility table
insert charging_station_position values('u',5.65501045464, 3.61291033625, -0.0804240616382, 0.996760738748);
insert charging_station_position values('v',-7.19262782348, 3.11045426516, 0.63173805188, 0.77518193594);
insert charging_station_position values('w',-23.6736662051, 5.65008294198, -0.52025554613,  0.854010636187);

-- Monday
insert into open_possibility_table values( 'a',2,'00:00:00','9:59:59',0,0);
insert into open_possibility_table values( 'a',2,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'a',2,'11:00:00','11:59:59',80,80);
insert into open_possibility_table values( 'a',2,'12:00:00','12:59:59',10,10);
insert into open_possibility_table values( 'a',2,'13:00:00','13:59:59',90,90);
insert into open_possibility_table values( 'a',2,'14:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'b',2,'00:00:00','11:59:59',0,0);
insert into open_possibility_table values( 'b',2,'12:00:00','12:59:59',90,90);
insert into open_possibility_table values( 'b',2,'13:00:00','13:59:59',100,100);
insert into open_possibility_table values( 'b',2,'14:00:00','14:59:59',90,90);
insert into open_possibility_table values( 'b',2,'15:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'c',2,'00:00:00','7:59:59',0,0);
insert into open_possibility_table values( 'c',2,'8:00:00','8:59:59',10,10);
insert into open_possibility_table values( 'c',2,'9:00:00','9:59:59',90,90);
insert into open_possibility_table values( 'c',2,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'c',2,'11:00:00','23:59:59',0,0);


-- Tuesday
insert into open_possibility_table values( 'b',3,'00:00:00','9:59:59',0,0);
insert into open_possibility_table values( 'b',3,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'b',3,'11:00:00','11:59:59',80,80);
insert into open_possibility_table values( 'b',3,'12:00:00','12:59:59',10,10);
insert into open_possibility_table values( 'b',3,'13:00:00','13:59:59',90,90);
insert into open_possibility_table values( 'b',3,'14:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'c',3,'00:00:00','11:59:59',0,0);
insert into open_possibility_table values( 'c',3,'12:00:00','12:59:59',90,90);
insert into open_possibility_table values( 'c',3,'13:00:00','13:59:59',100,100);
insert into open_possibility_table values( 'c',3,'14:00:00','14:59:59',90,90);
insert into open_possibility_table values( 'c',3,'15:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'a',3,'00:00:00','7:59:59',0,0);
insert into open_possibility_table values( 'a',3,'8:00:00','8:59:59',10,10);
insert into open_possibility_table values( 'a',3,'9:00:00','9:59:59',90,90);
insert into open_possibility_table values( 'a',3,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'a',3,'11:00:00','23:59:59',0,0);

-- Wednesday
insert into open_possibility_table values( 'a',4,'00:00:00','9:59:59',0,0);
insert into open_possibility_table values( 'a',4,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'a',4,'11:00:00','11:59:59',80,80);
insert into open_possibility_table values( 'a',4,'12:00:00','12:59:59',10,10);
insert into open_possibility_table values( 'a',4,'13:00:00','13:59:59',90,90);
insert into open_possibility_table values( 'a',4,'14:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'b',4,'00:00:00','11:59:59',0,0);
insert into open_possibility_table values( 'b',4,'12:00:00','12:59:59',90,90);
insert into open_possibility_table values( 'b',4,'13:00:00','13:59:59',100,100);
insert into open_possibility_table values( 'b',4,'14:00:00','14:59:59',90,90);
insert into open_possibility_table values( 'b',4,'15:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'c',4,'00:00:00','7:59:59',0,0);
insert into open_possibility_table values( 'c',4,'8:00:00','8:59:59',10,10);
insert into open_possibility_table values( 'c',4,'9:00:00','9:59:59',90,90);
insert into open_possibility_table values( 'c',4,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'c',4,'11:00:00','23:59:59',0,0);


-- Tursday
insert into open_possibility_table values( 'b',5,'00:00:00','9:59:59',0,0);
insert into open_possibility_table values( 'b',5,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'b',5,'11:00:00','11:59:59',80,80);
insert into open_possibility_table values( 'b',5,'12:00:00','12:59:59',10,10);
insert into open_possibility_table values( 'b',5,'13:00:00','13:59:59',90,90);
insert into open_possibility_table values( 'b',5,'14:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'c',5,'00:00:00','11:59:59',0,0);
insert into open_possibility_table values( 'c',5,'12:00:00','12:59:59',90,90);
insert into open_possibility_table values( 'c',5,'13:00:00','13:59:59',100,100);
insert into open_possibility_table values( 'c',5,'14:00:00','14:59:59',90,90);
insert into open_possibility_table values( 'c',5,'15:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'a',5,'00:00:00','7:59:59',0,0);
insert into open_possibility_table values( 'a',5,'8:00:00','8:59:59',10,10);
insert into open_possibility_table values( 'a',5,'9:00:00','9:59:59',90,90);
insert into open_possibility_table values( 'a',5,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'a',5,'11:00:00','23:59:59',0,0);

-- Friday
insert into open_possibility_table values( 'a',6,'00:00:00','9:59:59',0,0);
insert into open_possibility_table values( 'a',6,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'a',6,'11:00:00','11:59:59',80,80);
insert into open_possibility_table values( 'a',6,'12:00:00','12:59:59',10,10);
insert into open_possibility_table values( 'a',6,'13:00:00','13:59:59',90,90);
insert into open_possibility_table values( 'a',6,'14:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'b',6,'00:00:00','11:59:59',0,0);
insert into open_possibility_table values( 'b',6,'12:00:00','12:59:59',90,90);
insert into open_possibility_table values( 'b',6,'13:00:00','13:59:59',100,100);
insert into open_possibility_table values( 'b',6,'14:00:00','14:59:59',90,90);
insert into open_possibility_table values( 'b',6,'15:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'c',6,'00:00:00','7:59:59',0,0);
insert into open_possibility_table values( 'c',6,'8:00:00','8:59:59',10,10);
insert into open_possibility_table values( 'c',6,'9:00:00','9:59:59',90,90);
insert into open_possibility_table values( 'c',6,'10:00:00','10:59:59',90,90);
insert into open_possibility_table values( 'c',6,'11:00:00','23:59:59',0,0);

-- Weekend
insert into open_possibility_table values( 'a',7,'00:00:00','23:59:59',0,0);
insert into open_possibility_table values( 'b',7,'00:00:00','23:59:59',0,0);
insert into open_possibility_table values( 'c',7,'00:00:00','23:59:59',0,0);
insert into open_possibility_table values( 'd',7,'00:00:00','23:59:59',0,0);

insert into open_possibility_table values( 'a',1,'00:00:00','23:59:59',0,0);
insert into open_possibility_table values( 'b',1,'00:00:00','23:59:59',0,0);
insert into open_possibility_table values( 'c',1,'00:00:00','23:59:59',0,0);
insert into open_possibility_table values( 'd',1,'00:00:00','23:59:59',0,0);

-- 
-- -- -- create raw data with datetime and door status, 
call createRawData('a','2020-06-01 8:00:00','00:30:00',500);
call createRawData('b','2020-06-01 8:00:00','00:30:00',500);
call createRawData('c','2020-06-01 8:00:00','00:30:00',500);

select * from door_status_list;
select * from open_possibility_table;
select * from door_position;
select * from charging_station_position;