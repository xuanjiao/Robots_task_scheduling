DELETE FROM origin_db.tasks WHERE task_type <>'ExecuteTask';
SELECT * FROM origin_db.tasks;
SELECT  @exp_no;

-- Set up next experiment
SET @exp_no := @exp_no + 1; 

SET @LAST_TIME := '2020-06-01 9:00:00';
CALL origin_db.create_execute_tasks(@task_per_exp,@LAST_TIME,@LAST_TASK);


INSERT INTO origin_db.exe_weight
SELECT wt_btr,wt_wait,wt_psb, wt_pri FROM exp_db.exe_rs WHERE exp_no = 1;

