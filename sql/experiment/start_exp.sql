
-- Load backUp tables


-- Create execute task experiment result table
DROP TABLE IF EXISTS exp_db.exe_rs;
CREATE TABLE exp_db.exe_rs(
	exp_no INT AUTO_INCREMENT,
	wt_btr DOUBLE (5,2),
	wt_wait DOUBLE (5,2),
    wt_psb DOUBLE (5,2),
    wt_pri DOUBLE (5,2),
    start_time DATETIME,
    finish_time DATETIME,
    total INT,
    completed INT ,
    expired INT , 
	canceled INT,
    running INT,
    err INT,
    to_rerun INT,
    PRIMARY KEY (exp_no)
);

-- Make experiment cases

INSERT INTO exp_db.exe_rs
(wt_btr,wt_wait,wt_psb, wt_pri)
VALUES
(1,		1,		-1,		-1),
(10,	1,		-1,  	-1),
(1,		10,		-1,  	-1),
(1,		1,		-10,  	-1),
(1,		1,		-1, 	-10),
(0.1,	1,		-1,  	-1),
(1,		0.1,	-1,  	-1),
(1, 	1,		-0.1, 	-1),
(1,		1,		-1,		-0.1),
(10,	10,		-1,		-1),
(1,		10,		-10,	-1),
(1,		1,		-10,	-10),
(10,	10,		-0.1,	-0.1),
(0.1,	10,		-10,	-0.1),
(0.1,	0.1,	-10,	-10);

	
-- set task per experiment
SET @task_per_exp :=15;
SET @exp_no :=1;
-- Clear weight table. Insert weght 
TRUNCATE origin_db.exe_weight;

INSERT INTO origin_db.exe_weight
SELECT wt_btr,wt_wait,wt_psb, wt_pri FROM exp_db.exe_rs WHERE exp_no = 1;

-- Create first round of execute tasks
TRUNCATE origin_db.tasks;
SET @LAST_TASK := 0;
SET @LAST_TIME := '2020-06-01 9:00:00';
CALL origin_db.create_execute_tasks(@task_per_exp,@LAST_TIME,@LAST_TASK);

DROP EVENT 	IF EXISTS	exp_finish;		

delimiter ;;
																																					
CREATE EVENT exp_finish
ON SCHEDULE EVERY 10 MINUTE
DO BEGIN
	-- Split tasks to experiment 1,2,3 ... write experiment result in exec_task result 
    

    INSERT INTO exp_db.execute_tasks(task_type, start_time, target_id, robot_id , priority ,cur_status, dependency, description)
	SELECT  task_type, start_time, target_id, robot_id , priority ,cur_status, dependency, description FROM origin_db.tasks WHERE task_type = 'ExecuteTask';
    
    DELETE FROM origin_db.tasks;
    
    -- update weight table
    DELETE FROM origin_db.exe_weight;
    INSERT INTO origin_db.exe_weight
		SELECT wt_btr,wt_wait,wt_psb, wt_pri FROM exp_db.exe_rs  WHERE exp_no = @exp_no;
	
        -- get last task timestamp t, create next round of tests
	 CALL origin_db.create_execute_tasks(@task_per_exp,@LAST_TIME,@LAST_TASK);
     
	SET @exp_no := @exp_no + 1;
END;;

/*
CREATE TRIGGER  origin_db.new_weight
AFTER INSERT ON origin_db.exe_weight FOR EACH ROW
BEGIN


    -- get last task timestamp t, create next round of tests
	CALL origin_db.create_execute_tasks(@task_per_exp,@LAST_TIME,@LAST_TASK);
       
END;;
*/
DELIMITER ;

-- CALL origin_db.create_execute_tasks(@task_per_exp,@LAST_TIME,@LAST_TASK);
SELECT *  FROM exp_db.exe_rs;
SELECT * FROM origin_db.exe_weight;
SELECT * FROM origin_db.tasks;
SELECT @LAST_TASK;
SELECT @LAST_TIME;

SHOW EVENTS;
