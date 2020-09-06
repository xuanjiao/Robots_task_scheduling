
-- Create execute task experiment result table

DROP TABLE IF EXISTS exp_db.exe_rs;
CREATE TABLE exp_db.exe_rs(
	exp_no INT AUTO_INCREMENT,
	wt_btr DOUBLE (5,2),
	wt_wait DOUBLE (5,2),
    wt_psb DOUBLE (5,2),
    wt_pri DOUBLE (5,2),

    total INT DEFAULT 0,
    completed INT DEFAULT 0,
    not_run INT DEFAULT 0, 
	canceled INT DEFAULT 0 ,
    running INT DEFAULT 0,
    err INT DEFAULT 0 ,
    to_rerun INT DEFAULT 0 ,
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
SET @task_per_exp :=20;

-- Clear weight table. Insert weght 
TRUNCATE origin_db.exe_weight;

INSERT INTO origin_db.exe_weight
SELECT wt_btr,wt_wait,wt_psb, wt_pri FROM exp_db.exe_rs WHERE exp_no = 1;

-- Create first round of execute tasks

SET @start_time = '2020-06-01 9:00:00';
CALL origin_db.create_execute_tasks(@task_per_exp, @start_time +INTERVAL 30 SECOND);

SELECT * FROM exp_db.exe_rs;
SELECT * FROM origin_db.exe_weight;
