
-- Load backUp tables
TRUNCATE origin_db.charging_stations;
INSERT INTO origin_db.charging_stations(station_id)
VALUES
(18),(19),(20);


-- Create execute task experiment result table
DROP TABLE IF EXISTS origin_db.exe_rs;
CREATE TABLE origin_db.exe_rs(
	exp_id INT AUTO_INCREMENT,
	wt_btr DOUBLE (5,2),
	wt_wait DOUBLE (5,2),
    wt_psb DOUBLE (5,2),
    wt_pri DOUBLE (5,2),
    start_time DATETIME,
    finish_time DATETIME,
    duration TIME,
    total INT,
    completed INT ,
    expired INT , 
	canceled INT,
    running INT,
    err INT,
    to_rerun INT,
    PRIMARY KEY (exp_id)
);

-- Make experiment cases
TRUNCATE origin_db.exe_rs;

INSERT INTO origin_db.exe_rs
(wt_btr,wt_wait,wt_psb, wt_pri)
VALUES
(20,	1,		-1,		-1),
(40,	1,		-1,		-1),
(60,	1,		-1,		-1),
(80,	1,		-1,		-1),
(100,	1,		-1,		-1),

(1,		20,		-1,		-1),
(1,		40,		-1,  	-1),
(1,		60,		-1,  	-1),
(1,		80,		-1,  	-1),
(1,		100,	-1,  	-1),

(1,		1,		-20,	-1),
(1,		1,		-40,  	-1),
(1,		1,		-60,  	-1),
(1,		1,		-80,  	-1),
(1,		1,		-100,  	-1),

(1,		1,		-1,		-20),
(1,		1,		-1, 	-40),
(1,		1,		-1, 	-60),
(1,		1,		-1, 	-80),
(1,		1,		-1, 	-100);

TRUNCATE origin_db.tasks;
TRUNCATE origin_db.exe_weight;

-- 
INSERT tasks (task_type,target_id,robot_id,priority)
VALUES
('Charging',18,1,5),
('Charging',19,2,5),
('Charging',20,3,5);

-- set task per experiment
SET @task_per_exp :=18;
SET @exp_no := 1 ;

-- CALL origin_db.create_execute_tasks(@task_per_exp,@LAST_TIME,@LAST_TASK);
SELECT *  FROM origin_db.exe_rs;
SELECT * FROM origin_db.exe_weight;
SELECT * FROM origin_db.tasks;
SELECT * FROM origin_db.charging_stations;

SHOW EVENTS;

/*

INSERT INTO origin_db.exe_rs
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
*/