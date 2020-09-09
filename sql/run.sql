create database if not exists origin_db character set utf8 collate utf8_general_ci;
create user if not exists 'centralized_pool'@'localhost' identified by 'pass';
create user if not exists 'door_simulator'@'localhost' identified by 'pass';
create user if not exists 'sql_test_1'@'localhost' identified by 'pass';
create user if not exists 'sql_test_2'@'localhost' identified by 'pass';
create user if not exists 'charging_station'@'localhost' identified by 'pass';

grant all on origin_db.* to 'centralized_pool'@'localhost';
grant all on origin_db.* to 'door_simulator'@'localhost';
grant all on origin_db.* to 'task_generator'@'localhost';
grant all on origin_db.* to 'charging_station'@'localhost';
grant all on origin_db.* to 'sql_test_1'@'localhost';
grant all on origin_db.* to 'sql_test_2'@'localhost';

SET GLOBAL event_scheduler = ON;
SET SQL_SAFE_UPDATES = 0;

USE origin_db;

--   ------ Create robot table ---

DROP TABLE IF EXISTS origin_db.robots;
CREATE TABLE origin_db.robots(
	robot_id INT,
    robot_status ENUM('GatherInviromentInfo','ExecuteTask','Charging')
);

-- ----------Create room range table ----------------
DROP TABLE IF EXISTS origin_db.room_range;
CREATE TABLE origin_db.room_range (
	room_id INT,
    x_min DOUBLE (6,2),
    x_max DOUBLE (6,2),
    y_min DOUBLE (6,2),
    y_max DOUBLE (6,2),
    PRIMARY KEY (room_id)
);

INSERT INTO origin_db.room_range
VALUES
(0,-26.0,6.0,1.5,5.8),
(1,-24.0,-14.5,5.8,8.0),
(2,-26.0,-24.0,6.3,8.0),
(3,-26.0,-22.5,8.0,15.0),
(4,-22.5,-17.0,8.0,15.0),
(5,-17.0,-13.0,8.0,15.0),
(6,-14.5,-13.0,6.3,8.0),

(8,-12.0,-11.0,6.3,9.0),
(9,4.5,-0.5,5.8,8.5),
(7,-12.2,-0.5,5.8,15.0),

(10,2.8,10.0,5.8,8.0),
(11,1.2,2.8,7.3,8.0),
(12,1.2,5.0,8.0,15.0),
(13,5.0,10.0,8.0,15.0),

(14,6.0,14.0,1.5,5.8),

(16,-6.2,6.0,-1.8,1.5),
(15,-6.2,6.0,-8.5,1.5);




-- ----------Create room range table finished ----------------

-- ----------Create position table ----------------
DROP TABLE IF EXISTS origin_db.positions;
CREATE TABLE origin_db.positions (
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
INSERT INTO origin_db.positions 
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
(19,'ChargingStation',-21.0,5.0),
(21,'Point',-24.0,12.0),
(22,'Point',-21.0,12.0),
(23,'Point',-16.0,12.0),
(24,'Point',-7.0,7.5),
(25,'Point',-3.0,12),
(26,'Point',3.0,12.0),
(27,'Point',7.0,12.0),
(28,'Point',9.0,4.0),
(29,'Point',0.0,-4.0),
(30,'Point',-7.0,3.0);


-- Create door info table -------

DROP TABLE IF EXISTS origin_db.doors;
CREATE TABLE origin_db.doors(
	door_id INT REFERENCES positions(target_id),
    dependency INT REFERENCES positions(target_id),
    last_update DATETIME DEFAULT '2020-06-01 9:00:00',
    is_used BOOLEAN DEFAULT false,
    PRIMARY KEY (door_id)	
);	


INSERT INTO origin_db.doors(door_id,dependency)
VALUES
(1,0), (2,1), (3,1), (4,1), (5,1), (6,1), (7,0), (8,7), (9,0), (10,0), (11,10), (12,10), (13,10), (14,0), (15,0), (16,0);

-- Create door info table finished-------

-- Create charging station table --------

DROP TABLE IF EXISTS  origin_db.charging_stations;
CREATE TABLE origin_db.charging_stations(
	station_id INT REFERENCES positions(target_id),
    robot_battery_level DOUBLE(6,2) DEFAULT 100,
    charging_rate DOUBLE(6,2) DEFAULT 2,
    remaining_time DOUBLE(6,2) DEFAULT 0,
    PRIMARY KEY (station_id)
);

INSERT INTO origin_db.charging_stations(station_id)
SELECT target_id FROM origin_db.positions WHERE target_type = 'ChargingStation';

-- Create charging station table finished--------


DROP TABLE IF EXISTS origin_db.custom_points;
CREATE TABLE IF NOT EXISTS origin_db.custom_points (
	point_id INT REFERENCES positions(target_id),
    room_id INT REFERENCES room_range(room_id),
    PRIMARY KEY (point_id)
);


INSERT INTO origin_db.custom_points(point_id)
VALUES
(21),(22),(23),(24),(25),(26),(27),(28),(29),(30);

-- update custum point room_id	

UPDATE custom_points cp
INNER JOIN positions pos ON cp.point_id = pos.target_id 
INNER JOIN room_range r ON (pos.position_x BETWEEN r.x_min AND r.x_max )AND ( pos.position_y BETWEEN r.y_min AND y_max)
SET cp.room_id = r.room_id ;


-- Create possibility table --------
DROP TABLE IF EXISTS origin_db.open_possibilities;
CREATE TABLE origin_db.open_possibilities (
    door_id INT REFERENCES doors(door_id),
    day_of_week INT,
    start_time TIME,
    end_time TIME,
    open_pos DOUBLE(6,2),
    open_pos_st DOUBLE(6,2),
    CONSTRAINT Door_Time UNIQUE (day_of_week , start_time , end_time , door_id)
);


CALL create_possibility_table(16);

-- Create possibility table finished --------

-- Create measurement table --------

DROP TABLE IF EXISTS origin_db.measurements;
CREATE TABLE origin_db.measurements (
    door_id INT REFERENCES doors(door_id),
    door_status BOOLEAN,
    date_time DATETIME,
    CONSTRAINT Door_Date_Time UNIQUE (date_time , door_id)
);

CALL createRawData();
-- Create measurement table finished --------

-- Create task table --------

DROP TABLE IF EXISTS origin_db.tasks;
CREATE TABLE origin_db.tasks (
    task_id INT AUTO_INCREMENT,
    task_type ENUM('GatherEnviromentInfo', 'Charging','ExecuteTask'),
    start_time DATETIME,
    target_id INT,
    robot_id INT,
    priority INT,
    cur_status ENUM('Created', 'WaitingToRun', 'Running', 'RanToCompletion', 'Canceled','Error','ToReRun') DEFAULT 'Created' ,
    dependency INT,
    description varchar(255),
    PRIMARY KEY (task_id)																																																																								
);


-- Create task table finished --------


-- Create weight table --
DROP TABLE origin_db.exe_weight;
CREATE TABLE origin_db.exe_weight(
	wt_btr DOUBLE(6,2),
    wt_wait DOUBLE(6,2),  -- waiting time
    wt_psb DOUBLE(6,2),
    wt_pri DOUBLE(6,2)
);

DROP TABLE origin_db.door_weight;
CREATE TABLE origin_db.door_weight(
	wt_btr DOUBLE(6,2),
    wt_update DOUBLE(6,2), -- time since last update
    wt_psb DOUBLE(6,2),
    wt_used DOUBLE(6,2) -- is being used by another robot
);

DROP TABLE origin_db.charging_station_weight;
CREATE TABLE origin_db.charging_station_weight(
	wt_btr DOUBLE(6,2),  	-- battery level
    wt_remain DOUBLE(6,2)	-- time until finish charging
);


-- Create charging trigger, update info by time --------

DROP EVENT IF EXISTS origin_db.charging;    -- dynamic charging stations table
CREATE EVENT origin_db.charging
ON SCHEDULE EVERY 1 SECOND
STARTS CURRENT_TIMESTAMP ENDS CURRENT_TIMESTAMP + INTERVAL 3 HOUR
DO 
		UPDATE charging_stations 
        SET robot_battery_level = IF(robot_battery_level + charging_rate>=100,100,robot_battery_level + charging_rate),
			remaining_time =(100 - robot_battery_level)/charging_rate -- Accorging to charging rate, calculate robot_battery_level and charging time 
		WHERE robot_battery_level <100;

-- Create charging trigger finished --------

-- Create last update trigger, update last_update column --------

DROP TRIGGER IF EXISTS origin_db.last_update_trigger;
DELIMITER ;;
CREATE TRIGGER origin_db.last_update_trigger
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

DROP TRIGGER IF EXISTS origin_db.door_used;
DELIMITER ;;
CREATE TRIGGER origin_db.door_used
AFTER UPDATE ON origin_db.tasks FOR EACH ROW
BEGIN
	UPDATE origin_db.doors
    SET is_used = IF(NEW.cur_status = 'Running',1,0)
    where NEW.target_id = door_id;
END;
;;
DELIMITER ;

-- Create door_used finished --	


-- Print all tables --
SELECT * FROM origin_db.room_range;
SELECT * FROM origin_db.measurements;
SELECT * FROM origin_db.open_possibilities;
SELECT * FROM origin_db.positions;
SELECT * FROM origin_db.charging_stations;
SELECT * FROM origin_db.doors;
SELECT * FROM origin_db.tasks;
SELECT * FROM origin_db.custom_points;
