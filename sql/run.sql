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
    orientation_z DOUBLE DEFAULT 0,
    orientation_w DOUBLE DEFAULT 0,
    CONSTRAINT Pose UNIQUE(position_x,position_y,orientation_z,orientation_w),
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
    parent_task INT,
    child_task INT,
    result varchar(255),
    PRIMARY KEY (task_id)
);

-- insert value to targets table
INSERT INTO targets 
VALUES
-- (1,'Door',-21.6662127429, 9.50513223651, 9.50513223651, 0.667743723726),
-- (2,'Door',-19.4278817305, 9.0054257766, 0.673662651895, 0.739038991827),
-- (3,'Door',-14.7790163094, 9.0879504651, 0.696015604107, 0.718026656078),
-- (4,'Door',-22.3155041827, 8.20714003079, -0.999991890265,0.0040273322644),
-- (5,'Door',-17.6463588911, -17.6463588911, 0.710953141288, 0.703239383775),
-- (6,'Door',-13.2646228952, 7.77758415791, 7.77758415791,0.998654018142),
(7,'Door',-9.72013338947, 8.52865724571, -0.999999263125, 0.0012139806762),
(8,'Door',-9.03927424531, 6.62321407921, 0.651078250306, 0.759010613877),
(9,'Door',-0.868440756591, 7.0788002511, 0.662118779871, 0.749398906686),
(10,'Door',4.38077210276, 9.34650744461, 0.721488227349, 0.692426702112),
(11,'Door',6.48236977439, 9.48845367671, 0.711113291282,0.703077440231),
(12,'Door',3.6025773004, 8.71864545073, -0.99768098292, 0.0680636196487),
(13,'Door',4.23119675634, 7.38657063027, 0.608479240054, 0.793569791778),
(14,'Door',6.86290545247, 5.40794760884, 0.114759478636, 0.993393306834),
(15,'Door',-2.95220390532, 2.39211836475, -0.702331560869, -0.702331560869),

(16,'ChargingStation',5.65501045464, 3.61291033625, -0.0804240616382, 0.996760738748),
(17,'ChargingStation',-7.19262782348, 3.11045426516, 0.63173805188, 0.77518193594),
(18,'ChargingStation',-23.6736662051, 5.65008294198, -0.52025554613,  0.854010636187);

-- insert charging station to table
INSERT INTO charging_stations(station_id)
SELECT target_id FROM targets WHERE target_type = 'ChargingStation';
 
-- fill in status list and possibility table
call createPossibilityTable(15);

SELECT * FROM door_status;
SELECT * FROM open_possibilities;
SELECT * FROM targets;
SELECT * FROM charging_stations;
