-- TRUNCATE exp_db.execute_tasks;

INSERT INTO exp_db.execute_tasks(task_type, start_time, target_id, robot_id , priority ,cur_status, dependency, description)
SELECT  task_type, start_time, target_id, robot_id , priority ,cur_status, dependency, description 
FROM origin_db.tasks 
WHERE task_type = 'ExecuteTask';

CALL exp_db.update_exp_result();
SELECT * FROM origin_db.tasks;
SELECT * FROM exp_db.execute_tasks;
SELECT * FROM origin_db.exe_weight;
SELECT *  FROM exp_db.exe_rs;

