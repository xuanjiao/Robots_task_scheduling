SELECT * FROM origin_db.tasks;

SET @last_task :=22; -- From task table, last running task is 22

-- DROP TABLE exp_db.result_1;
CREATE TABLE exp_db.result_3
SELECT 
	cur_status as last_status, COUNT(task_id) as cnt
    FROM origin_db.tasks 
	WHERE task_id <=  @last_task AND task_type = 'ExecuteTask'
    GROUP BY cur_status;
    
SELECT * FROM exp_db.result_3;