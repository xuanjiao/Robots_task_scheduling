create database if not exists sensor_db character set utf8 collate utf8_general_ci;
create user if not exists 'centralized_pool'@'localhost' identified by 'pass';
create user if not exists 'door_simulator'@'localhost' identified by 'pass';
create user if not exists 'task_generator'@'localhost' identified by 'pass';

grant all on sensor_db.* to 'centralized_pool'@'localhost';
grant all on sensor_db.* to 'door_simulator'@'localhost';
grant all on sensor_db.* to 'task_generator'@'localhost';

use sensor_db;

drop table if exists open_possibilities;
CREATE TABLE open_possibilities (
    door_id INT,
    day_of_week INT,
    start_time TIME,
    end_time TIME,
    open_pos DOUBLE(4 , 1 ),
    open_pos_st DOUBLE(4 , 1 ),
    CONSTRAINT Door_Time UNIQUE (day_of_week , start_time , end_time , door_id)
);

drop table if exists door_status;
CREATE TABLE door_status (
    door_id INT,
    door_status BOOLEAN,
    date_time DATETIME,
    CONSTRAINT Door_Date_Time UNIQUE (date_time , door_id)
);

drop table if exists targets;
CREATE TABLE targets (
    target_id INT AUTO_INCREMENT,
    target_type varchar(255) NULL,
    position_x DOUBLE DEFAULT 0,
    position_y DOUBLE DEFAULT 0,
    -- orientation_z DOUBLE DEFAULT 0,
    -- orientation_w DOUBLE DEFAULT 0,
    -- CONSTRAINT Pose UNIQUE(position_x,position_y,orientation_z,orientation_w),
    CONSTRAINT Pose UNIQUE(position_x,position_y),
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
    cur_status ENUM('Created', 'WaitingToRun', 'Running', 'RanToCompletion', 'Canceled','Error','ToReRun') DEFAULT 'Created',
    dependency INT,
    result varchar(255),
    PRIMARY KEY (task_id)
);

-- insert value to targets table
INSERT INTO targets 
VALUES
(1,'Door',-18.5,5.2),
(2,'Door',-23.5,7.2),
(3,'Door',-23.0,8.5),
(4,'Door',-20.5,7.7),
(5,'Door',-16.0,7.7),
(6,'Door',-15.0,6.8),
(7,'Door',-10.3,5.2),
(8,'Door',-10.3,7.5),
(9,'Door',-1.5,5.2),
(10,'Door',3.2,5.2),
(11,'Door',3.0,7.2),
(12,'Door',3.5,7.7),
(13,'Door',5.8,7.7),
(14,'Door',6.0,4.0),
(15,'Door',3.2,1.5),
(16,'Door',-4.0,1.5),
(17,'ChargingStation',0.0,5.0),
(18,'ChargingStation',-7.0,5.0),
(19,'ChargingStation',-21.0,5.0);


-- insert charging station to table
INSERT INTO charging_stations(station_id)
SELECT target_id FROM targets WHERE target_type = 'ChargingStation';
 
-- fill in status list and possibility table
call createPossibilityTable(15);

SELECT * FROM door_status;
SELECT * FROM open_possibilities;
SELECT * FROM targets;
SELECT * FROM charging_stations;
