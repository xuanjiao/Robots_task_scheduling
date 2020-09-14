-- ---Create charging task for robots 
SELECT * FROM origin_db.tasks;
INSERT tasks (task_type,target_id,robot_id,priority)
VALUES
('Charging',17,0,5),
('Charging',18,1,5),
('Charging',19,2,5);

-- DELETE FROM origin_db.tasks WHERE task_id IN (16,17,18);


UPDATE tasks SET finish_time = '2020-06-01 9:05:30', cur_status = 'RanToCompletion' WHERE task_id = 15;

-- Simulate centralized pool update task status and end time
UPDATE tasks SET finish_time = '2020-06-01 9:06:00' , cur_status = 'RanToCompletion' WHERE task_id = 16;
UPDATE tasks SET finish_time = '2020-06-01 9:06:30' , cur_status = 'RanToCompletion' WHERE task_id = 17;
UPDATE tasks SET finish_time = '2020-06-01 9:06:40' , cur_status = 'RanToCompletion' WHERE task_id = 18;

-- ---Update experiment start time

SELECT * FROM tasks t INNER JOIN positions p ON t.target_id = p.target_id 
      WHERE task_type = 'Charging' AND robot_id = 0;
      
SELECT * FROM origin_db.exe_weight;
SELECT * FROM origin_db.exe_weight;

      SELECT t.position_x, t.position_y, cs.station_id, cs.cur_status, cs.battery, TIME_TO_SEC(cs.remaining_time) AS t 
      FROM charging_stations cs 
      INNER JOIN positions t ON t.target_id = cs.station_id