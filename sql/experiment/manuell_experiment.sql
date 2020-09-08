
DROP TABLE IF EXISTS exp_db.exe_rs;
CREATE TABLE exp_db.exe_rs(
	exp_no INT AUTO_INCREMENT,
	wt_btr DOUBLE (5,2),
	wt_wait DOUBLE (5,2),
    wt_psb DOUBLE (5,2),
    wt_pri DOUBLE (5,2),
    last_task_finish_time DATETIME,
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
/*

DROP TABLE exp_db.execute_tasks;
CREATE TABLE exp_db.execute_tasks(
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
*/

-- set task per experiment
SET @task_per_exp :=15;

-- Clear weight table. Insert weght 
TRUNCATE origin_db.exe_weight;
TRUNCATE origin_db.tasks;
SET @LAST_TASK := 0;
SET @LAST_TIME := '2020-06-01 9:00:00';
CALL origin_db.create_execute_tasks(@task_per_exp,@LAST_TIME,@LAST_TASK);

SET @exp_no := 15 ;
INSERT INTO origin_db.exe_weight
SELECT wt_btr,wt_wait,wt_psb, wt_pri FROM exp_db.exe_rs WHERE exp_no = @exp_no;


SELECT *  FROM exp_db.exe_rs;
SELECT * FROM origin_db.exe_weight;
SELECT * FROM origin_db.tasks;
SELECT @exp_no;


