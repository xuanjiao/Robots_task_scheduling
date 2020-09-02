create database if not exists sensor_db character set utf8 collate utf8_general_ci;
create user if not exists 'centralized_pool'@'localhost' identified by 'pass';
create user if not exists 'door_simulator'@'localhost' identified by 'pass';
create user if not exists 'task_generator'@'localhost' identified by 'pass';
create user if not exists 'charging_station'@'localhost' identified by 'pass';

grant all on sensor_db.* to 'centralized_pool'@'localhost';
grant all on sensor_db.* to 'door_simulator'@'localhost';
grant all on sensor_db.* to 'task_generator'@'localhost';
grant all on sensor_db.* to 'charging_station'@'localhost';

SET GLOBAL event_scheduler = ON;
SET SQL_SAFE_UPDATES = 0;

use sensor_db;

drop table if exists positions;
CREATE TABLE positions (
    target_id INT AUTO_INCREMENT,
    target_type varchar(255) NULL,
    position_x DOUBLE (6,2)DEFAULT 0,
    position_y DOUBLE (6,2)DEFAULT 0,
    -- orientation_z DOUBLE DEFAULT 0,
    -- orientation_w DOUBLE DEFAULT 0,
    -- CONSTRAINT Pose UNIQUE(position_x,position_y,orientation_z,orientation_w),
    CONSTRAINT Pose UNIQUE(position_x,position_y),
    PRIMARY KEY (target_id)
);

-- insert value to positions table
INSERT INTO positions 
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
(18,'ChargingStation',-21.0,5.0),
(21,'Point',-24.0,12.0),
(22,'Point',-21.0,12.0),
(23,'Point',-16.0,12.0),
(24,'Point',-7.0,7.5),
(25,'Point',-3.0,12),
(26,'Point',3.0,12.0),
(27,'Point',7.0,12.0),
(28,'Point',9.0,4.0),
(29,'Point',0.0,-4.0);


-- Create door info table -------

DROP TABLE IF EXISTS doors;
CREATE TABLE doors(
	door_id INT REFERENCES positions(target_id),
    dependency INT REFERENCES positions(target_id),
    last_update DATETIME DEFAULT '2020-06-01 9:00:00',
    is_used BOOLEAN DEFAULT false,
    PRIMARY KEY (door_id)	
);	


INSERT INTO doors(door_id,dependency)
VALUES
(1,0), (2,1), (3,1), (4,1), (5,1), (6,1), (7,0), (8,7), (9,0), (10,0), (11,10), (12,10), (13,10), (14,0), (15,0), (16,0);

-- Create door info table finished-------

-- Create charging station table --------

DROP TABLE IF EXISTS  charging_stations;
CREATE TABLE charging_stations(
	station_id INT REFERENCES positions(target_id),
    robot_battery_level DOUBLE(6,2) DEFAULT 100,
    charging_rate DOUBLE(6,2) DEFAULT 2,
    remaining_time DOUBLE(6,2) DEFAULT 0,
    PRIMARY KEY (station_id)
);

INSERT INTO charging_stations(station_id)
SELECT target_id FROM positions WHERE target_type = 'ChargingStation';

-- Create charging station table finished--------


DROP TABLE IF EXISTS custom_points;
CREATE TABLE IF NOT EXISTS custom_points (
	point_id INT REFERENCES positions(target_id),
    door_id INT DEFAULT 0,
    PRIMARY KEY (point_id)
);
INSERT INTO custom_points
VALUES
(21,3),(22,4),(23,5),(24,7),(25,7),(26,10),(27,13),(28,14),(29,15),(30,0);


-- Create possibility table --------
drop table if exists open_possibilities;
CREATE TABLE open_possibilities (
    door_id INT REFERENCES doors(door_id),
    day_of_week INT,
    start_time TIME,
    end_time TIME,
    open_pos DOUBLE(6,2),
    open_pos_st DOUBLE(6,2),
    CONSTRAINT Door_Time UNIQUE (day_of_week , start_time , end_time , door_id)
);


CALL createPossibilityTable(16);

-- Create possibility table finished --------

-- Create measurement table --------

DROP TABLE IF EXISTS measurements;
CREATE TABLE measurements (
    door_id INT REFERENCES doors(door_id),
    door_status BOOLEAN,
    date_time DATETIME,
    CONSTRAINT Door_Date_Time UNIQUE (date_time , door_id)
);

CALL createRawData();
-- Create measurement table finished --------

-- Create task table --------

DROP TABLE IF EXISTS tasks;
CREATE TABLE tasks (
    task_id INT AUTO_INCREMENT,
    task_type ENUM('GatherEnviromentInfo', 'Charging','ExecuteTask'),
    start_time DATETIME,
    target_id INT,
    robot_id INT,
    priority INT,
    cur_status varchar(255) DEFAULT 'Created',
    dependency INT,
    description varchar(255),
    PRIMARY KEY (task_id)																																																																								
);

-- ENUM('Created', 'WaitingToRun', 'Running', 'RanToCompletion', 'Canceled','Error','ToReRun') 


-- Create task table finished --------


-- Create charging trigger, update info by time --------

DROP EVENT IF EXISTS charging;    -- dynamic charging stations table
CREATE EVENT charging
ON SCHEDULE EVERY 1 SECOND
STARTS CURRENT_TIMESTAMP ENDS CURRENT_TIMESTAMP + INTERVAL 3 HOUR
DO 
		UPDATE charging_stations 
        SET robot_battery_level = IF(robot_battery_level + charging_rate>=100,100,robot_battery_level + charging_rate),
			remaining_time =(100 - robot_battery_level)/charging_rate -- Accorging to charging rate, calculate robot_battery_level and charging time 
		WHERE robot_battery_level <100;

-- Create charging trigger finished --------

-- Create last update trigger, update last_update column --------

DROP TRIGGER IF EXISTS last_update_trigger;
DELIMITER ;;
CREATE TRIGGER last_update_trigger
AFTER INSERT ON measurements FOR EACH ROW
BEGIN
	UPDATE doors
    SET last_update = NEW.date_time
    WHERE door_id = NEW.door_id;
END;
;;
DELIMITER ;

-- Create last update trigger ---

-- Create door_used trigger --

DROP TRIGGER IF EXISTS door_used;
DELIMITER ;;
CREATE TRIGGER door_used
AFTER UPDATE ON tasks FOR EACH ROW
BEGIN
	UPDATE doors
    SET is_used = IF(NEW.cur_status = 'Running',1,0)
    where NEW.target_id = door_id;
END;
;;
DELIMITER ;

-- Create door_used finished --		

call create_execute_tasks();

-- Print all tables --
SELECT * FROM measurements;
SELECT * FROM open_possibilities;
SELECT * FROM positions;
SELECT * FROM charging_stations;
SELECT * FROM doors;
SELECT * FROM tasks;
SELECT * FROM custom_points;
