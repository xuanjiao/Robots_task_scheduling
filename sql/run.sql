create database if not exists sensor_db character set utf8 collate utf8_general_ci;
create user if not exists 'centralized_pool'@'localhost' identified by 'pass';
create user if not exists 'door_simulator'@'localhost' identified by 'pass';

grant all on sensor_db.* to 'centralized_pool'@'localhost';
grant all on sensor_db.* to 'door_simulator'@'localhost';

use sensor_db;

drop table if exists open_possibilities;
CREATE TABLE open_possibilities (
    door_id INT,
    day_of_week INT,
    start_time TIME,
    end_time TIME,
    open_pos DOUBLE(4 , 1 ),
    open_pos_st DOUBLE(4 , 1 ),
    CONSTRAINT Door_Time PRIMARY KEY (day_of_week , start_time , end_time , door_id)
);

drop table if exists door_status;
CREATE TABLE door_status (
    door_id INT,
    door_status BOOLEAN,
    date_time DATETIME,
    CONSTRAINT Door_Date_Time PRIMARY KEY (date_time , door_id)
);

drop table if exists targets;
CREATE TABLE targets (
    target_id INT AUTO_INCREMENT,
    target_type ENUM('ChargingStation', 'Door','Point'),
    position_x DOUBLE DEFAULT 0,
    position_y DOUBLE DEFAULT 0,
    orientation_z DOUBLE DEFAULT 0,
    orientation_w DOUBLE DEFAULT 0,
    PRIMARY KEY (target_id)
);

DROP TABLE IF EXISTS  charging_stations;
CREATE TABLE charging_stations(
	station_id INT REFERENCES targets(target_id),
    is_free BOOLEAN DEFAULT true,
    PRIMARY KEY (station_id)
);

drop table if exists tasks;
CREATE TABLE tasks (
    task_id INT AUTO_INCREMENT,
    task_type ENUM('GatherEnviromentInfo', 'Charging','ExecuteTask'),
    start_time DATETIME,
    target_id INT,
    robot_id INT DEFAULT 0,
    priority INT DEFAULT 0,
    cur_status ENUM('Created', 'WaitingToRun', 'Running', 'RanToCompletion', 'Canceled','Error') DEFAULT 'Created',
    parent_task INT,
    child_task INT,
    result varchar(255),
    PRIMARY KEY (task_id)
);

drop table if exists costs;
CREATE TABLE costs (
    task_id INT,
    robot_id INT,
    priority INT,
    open_pos_st DOUBLE,
    time_diff DOUBLE,
    battery DOUBLE,
    cost DOUBLE,
     distance DOUBLE,
    PRIMARY KEY (task_id)
);

-- insert value to door_position table
-- insert door_position values(1,-21.6662127429, 9.50513223651, 9.50513223651, 0.667743723726);
-- insert door_position values(2,-19.4278817305, 9.0054257766, 0.673662651895, 0.739038991827);
-- insert door_position values(3,-14.7790163094, 9.0879504651, 0.696015604107, 0.718026656078);
-- insert door_position values('d',-22.3155041827, 8.20714003079, -0.999991890265,0.0040273322644);
-- insert door_position values('e',-17.6463588911, -17.6463588911, 0.710953141288, 0.703239383775);
-- insert door_position values('f',-13.2646228952, 7.77758415791, 7.77758415791,0.998654018142);
-- insert door_position values('g',-9.72013338947, 8.52865724571, -0.999999263125, 0.0012139806762);
-- insert door_position values('h',-9.03927424531, 6.62321407921, 0.651078250306, 0.759010613877);
-- insert door_position values('i',-0.868440756591, 7.0788002511, 0.662118779871, 0.749398906686);
-- insert door_position values('j',4.38077210276, 9.34650744461, 0.721488227349, 0.692426702112);
-- insert door_position values('k',6.48236977439, 9.48845367671, 0.711113291282,0.703077440231);
-- insert door_position values('l',3.6025773004, 8.71864545073, -0.99768098292, 0.0680636196487);

-- insert value to target list
insert targets values(1,'Door',4.23119675634, 7.38657063027, 0.608479240054, 0.793569791778);
insert targets values(2,'Door',6.86290545247, 5.40794760884, 0.114759478636, 0.993393306834);
insert targets values(3,'Door',-2.95220390532, 2.39211836475, -0.702331560869, -0.702331560869);
-- insert targets values('d','Door',4.28632121392, 2.54533955273, -0.723120814312, 0.690721570468);

insert targets values(4,'ChargingStation',5.65501045464, 3.61291033625, -0.0804240616382, 0.996760738748);
insert targets values(5,'ChargingStation',-7.19262782348, 3.11045426516, 0.63173805188, 0.77518193594);
insert targets values(6,'ChargingStation',-23.6736662051, 5.65008294198, -0.52025554613,  0.854010636187);

-- Monday
insert into open_possibilities values( 1,2,'00:00:00','7:59:59',0,0);
insert into open_possibilities values( 1,2,'08:00:00','9:59:59',0.8,0.8);
insert into open_possibilities values( 1,2,'10:00:00','10:59:59',0.9,0.9);
insert into open_possibilities values( 1,2,'11:00:00','11:59:59',0.8,0.8);
insert into open_possibilities values( 1,2,'12:00:00','12:59:59',0.1,0.1);
insert into open_possibilities values( 1,2,'13:00:00','16:59:59',0.9,0.9);
insert into open_possibilities values( 1,2,'17:00:00','23:59:59',0,0);

insert into open_possibilities values( 2,2,'00:00:00','8:59:59',0,0);
insert into open_possibilities values( 2,2,'9:00:00','9:59:59',0.9,0.9);
insert into open_possibilities values( 2,2,'10:00:00','13:59:59',1.0,1.0);
insert into open_possibilities values( 2,2,'14:00:00','17:59:59',0.9,0.9);
insert into open_possibilities values( 2,2,'18:00:00','23:59:59',0,0);

insert into open_possibilities values( 3,2,'00:00:00','7:59:59',0,0);
insert into open_possibilities values( 3,2,'8:00:00','8:59:59',0.1,0.1);
insert into open_possibilities values( 3,2,'9:00:00','9:59:59',0.9,0.9);
insert into open_possibilities values( 3,2,'10:00:00','15:59:59',0.9,0.9);
insert into open_possibilities values( 3,2,'16:00:00','23:59:59',0,0);


-- Tuesday
insert into open_possibilities values( 2,3,'00:00:00','9:59:59',0,0);
insert into open_possibilities values( 2,3,'10:00:00','10:59:59',0.9,0.9);
insert into open_possibilities values( 2,3,'11:00:00','11:59:59',0.8,0.8);
insert into open_possibilities values( 2,3,'12:00:00','12:59:59',0.1,0.1);
insert into open_possibilities values( 2,3,'13:00:00','13:59:59',0.9,0.9);
insert into open_possibilities values( 2,3,'14:00:00','23:59:59',0,0);

insert into open_possibilities values( 3,3,'00:00:00','11:59:59',0,0);
insert into open_possibilities values( 3,3,'12:00:00','12:59:59',0.9,0.9);
insert into open_possibilities values( 3,3,'13:00:00','13:59:59',1.0,1.0);
insert into open_possibilities values( 3,3,'14:00:00','16:59:59',0.9,0.9);
insert into open_possibilities values( 3,3,'17:00:00','23:59:59',0,0);

insert into open_possibilities values( 1,3,'00:00:00','7:59:59',0,0);
insert into open_possibilities values( 1,3,'8:00:00','8:59:59',0.9,0.9);
insert into open_possibilities values( 1,3,'9:00:00','9:59:59',1.0,1.0);
insert into open_possibilities values( 1,3,'10:00:00','10:59:59',0.9,0.9);
insert into open_possibilities values( 1,3,'11:00:00','23:59:59',0,0);

-- Wednesday
insert into open_possibilities values( 1,4,'00:00:00','9:59:59',0,0);
insert into open_possibilities values( 1,4,'10:00:00','10:59:59',0.9,0.9);
insert into open_possibilities values( 1,4,'11:00:00','11:59:59',0.8,0.8);
insert into open_possibilities values( 1,4,'12:00:00','12:59:59',0.1,0.1);
insert into open_possibilities values( 1,4,'13:00:00','15:59:59',0.9,0.9);
insert into open_possibilities values( 1,4,'16:00:00','23:59:59',0,0);

insert into open_possibilities values( 2,4,'00:00:00','11:59:59',0,0);
insert into open_possibilities values( 2,4,'12:00:00','12:59:59',0.9,0.9);
insert into open_possibilities values( 2,4,'13:00:00','13:59:59',1.0,1.0);
insert into open_possibilities values( 2,4,'14:00:00','15:59:59',0.9,0.9);
insert into open_possibilities values( 2,4,'16:00:00','23:59:59',0,0);

insert into open_possibilities values( 3,4,'00:00:00','7:59:59',0,0);
insert into open_possibilities values( 3,4,'8:00:00','8:59:59',0.1,0.1);
insert into open_possibilities values( 3,4,'9:00:00','9:59:59',0.9,0.9);
insert into open_possibilities values( 3,4,'10:00:00','10:59:59',0.9,0.9);
insert into open_possibilities values( 3,4,'11:00:00','23:59:59',0,0);


-- Tursday
insert into open_possibilities values( 2,5,'00:00:00','9:59:59',0,0);
insert into open_possibilities values( 2,5,'10:00:00','10:59:59',0.9,0.9);
insert into open_possibilities values( 2,5,'11:00:00','11:59:59',0.8,0.8);
insert into open_possibilities values( 2,5,'12:00:00','12:59:59',0.1,0.1);
insert into open_possibilities values( 2,5,'13:00:00','16:59:59',0.9,0.9);
insert into open_possibilities values( 2,5,'17:00:00','23:59:59',0,0);

insert into open_possibilities values( 3,5,'00:00:00','11:59:59',0,0);
insert into open_possibilities values( 3,5,'12:00:00','12:59:59',0.9,0.9);
insert into open_possibilities values( 3,5,'13:00:00','13:59:59',1.0,1.0);
insert into open_possibilities values( 3,5,'14:00:00','16:59:59',0.9,0.9);
insert into open_possibilities values( 3,5,'17:00:00','23:59:59',0,0);

insert into open_possibilities values( 1,5,'00:00:00','7:59:59',0,0);
insert into open_possibilities values( 1,5,'8:00:00','8:59:59',0.1,0.1);
insert into open_possibilities values( 1,5,'9:00:00','9:59:59',0.9,0.9);
insert into open_possibilities values( 1,5,'10:00:00','15:59:59',0.9,0.9);
insert into open_possibilities values( 1,5,'16:00:00','23:59:59',0,0);

-- Friday
insert into open_possibilities values( 1,6,'00:00:00','9:59:59',0,0);
insert into open_possibilities values( 1,6,'10:00:00','10:59:59',0.9,0.9);
insert into open_possibilities values( 1,6,'11:00:00','11:59:59',0.8,0.8);
insert into open_possibilities values( 1,6,'12:00:00','12:59:59',0.1,0.1);
insert into open_possibilities values( 1,6,'13:00:00','16:59:59',0.9,0.9);
insert into open_possibilities values( 1,6,'17:00:00','23:59:59',0,0);

insert into open_possibilities values( 2,6,'00:00:00','11:59:59',0,0);
insert into open_possibilities values( 2,6,'12:00:00','12:59:59',0.9,0.9);
insert into open_possibilities values( 2,6,'13:00:00','13:59:59',1.0,1.0);
insert into open_possibilities values( 2,6,'14:00:00','14:59:59',0.9,0.9);
insert into open_possibilities values( 2,6,'15:00:00','23:59:59',0,0);

insert into open_possibilities values( 3,6,'00:00:00','7:59:59',0,0);
insert into open_possibilities values( 3,6,'8:00:00','8:59:59',0.1,0.1);
insert into open_possibilities values( 3,6,'9:00:00','9:59:59',0.9,0.9);
insert into open_possibilities values( 3,6,'10:00:00','16:59:59',0.9,0.9);
insert into open_possibilities values( 3,6,'17:00:00','23:59:59',0,0);

-- Weekend
insert into open_possibilities values( 1,7,'00:00:00','23:59:59',0,0);
insert into open_possibilities values( 2,7,'00:00:00','23:59:59',0,0);
insert into open_possibilities values( 3,7,'00:00:00','23:59:59',0,0);

insert into open_possibilities values( 1,1,'00:00:00','23:59:59',0,0);
insert into open_possibilities values( 2,1,'00:00:00','23:59:59',0,0);
insert into open_possibilities values( 3,1,'00:00:00','23:59:59',0,0);

-- insert charging station to table
INSERT INTO charging_stations(station_id)
SELECT target_id FROM targets WHERE target_type = 'ChargingStation';

 
-- -- -- create raw data with datetime and door status, 
  call createRawData(1,'2020-06-01 8:00:00','00:10:00',50);
  call createRawData(2,'2020-06-01 8:00:00','00:10:00',50);
  call createRawData(3,'2020-06-01 8:00:00','00:10:00',50);
-- 


SELECT * FROM door_status;
SELECT * FROM open_possibilities;
SELECT * FROM targets;
SELECT * FROM charging_stations;
