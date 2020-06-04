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