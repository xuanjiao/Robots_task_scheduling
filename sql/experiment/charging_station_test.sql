-- ---Create charging task for robots 
SELECT * FROM origin_db.tasks;

INSERT tasks (task_type,target_id,robot_id,priority)
VALUES
('Charging',17,0,5),
('Charging',18,1,5),
('Charging',19,2,5);

-- DELETE FROM origin_db.tasks WHERE task_id IN (16,17,18);


-- UPDATE tasks SET finish_time = '2020-06-01 9:05:30', cur_status = 'RanToCompletion' WHERE task_id = 15;

-- Simulate centralized pool update task status and end time
UPDATE tasks SET finish_time = '2020-06-01 9:06:00' , cur_status = 'RanToCompletion' WHERE task_id = 1;
UPDATE tasks SET finish_time = '2020-06-01 9:06:30' , cur_status = 'RanToCompletion' WHERE task_id = 2;
UPDATE tasks SET finish_time = '2020-06-01 9:06:40' , cur_status = 'RanToCompletion' WHERE task_id = 3;


UPDATE origin_db.charging_stations
	SET robot_id = 0, battery = 80 WHERE station_id = 17;
    
UPDATE origin_db.charging_stations
    SET robot_id = 1, battery = 90 WHERE station_id = 18;
    
UPDATE origin_db.charging_stations
    SET robot_id = 2, battery = 93 WHERE station_id = 19;

SELECT * FROM origin_db.charging_stations;
-- 

SELECT * FROM origin_db.tasks;
SELECT * FROM origin_db.tasks;
SELECT * FROM exp_db.exe_rs;

UPDATE charging_stations SET battery = NULL, robot_id = NULL WHERE station_id = 17;
      

--  UPDATE origin_db.charging_stations  
--  SET robot_id  = NULL WHERE station_id = 17;
-- -- 
-- UPDATE origin_db.charging_stations  
--  SET robot_id  = NULL WHERE station_id = 18;
-- -- 
-- UPDATE origin_db.charging_stations  
-- SET robot_id  = NULL WHERE station_id = 19;