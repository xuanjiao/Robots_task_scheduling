-- ---Create charging task for robots 
SELECT * FROM origin_db.tasks;
INSERT tasks (task_type,target_id,robot_id,priority)
VALUES
('Charging',17,0,5),
('Charging',18,1,5),
('Charging',19,2,5);

-- DELETE FROM origin_db.tasks WHERE task_id IN (16,17,18);


UPDATE tasks SET finish_time = '2020-06-01 9:05:30', cur_status = 'RanToCompletion' WHERE task_id = 15;
CALL exp_db.update_exp_result();
-- Simulate centralized pool update task status and end time
UPDATE tasks SET finish_time = '2020-06-01 9:06:00' , cur_status = 'RanToCompletion' WHERE task_id = 16;
UPDATE tasks SET finish_time = '2020-06-01 9:06:30' , cur_status = 'RanToCompletion' WHERE task_id = 17;
UPDATE tasks SET finish_time = '2020-06-01 9:06:40' , cur_status = 'RanToCompletion' WHERE task_id = 18;

-- ---Update experiment start time

SET @task_per_exp = 18; -- 15 execute task 3 charging task;

UPDATE exp_db.exe_rs rs
INNER JOIN (
	SELECT 
 		FLOOR((t.task_id -1)/ @task_per_exp + 2) AS exp_no, 
        MAX(finish_time) AS st
	FROM origin_db.tasks t WHERE t.task_type = 'Charging'
 	GROUP BY exp_no
	) tmp ON rs.exp_no = tmp.exp_no
SET rs.start_time = tmp.st;

SELECT * FROM origin_db.tasks;
SELECT * FROM exp_db.exe_rs;
SELECT * FROM origin_db.charging_stations ;

	UPDATE origin_db.charging_stations 
    SET battery = IF(battery + charging_rate>=100 AND cur_status <> 'Free' ,100,battery + charging_rate),
	cur_status = IF (battery + charging_rate>=100 AND cur_status <> 'Free' , 'Charging_finish','Charging');
  
	-- If 3 robot Charging_finish -> Next_exp
    SELECT 
		SUM(IF(cur_status = 'Charging_finish', 1, 0)) ,
		SUM(IF(cur_status = 'Next_exp', 1, 0)) INTO @sumFinish, @sumNextExp
	FROM origin_db.charging_stations;
    
    SELECT @sumFinish, @sumNextExp;
   
    IF (sumFinish = 3 )THEN
		UPDATE origin_db.charging_stations	SET cur_status = 'Next_exp';
		CALL origin_db.exp_db.update_exp_result();
		CALL origin_db.exp_db.update_next_exp_start_time();
    END IF;

