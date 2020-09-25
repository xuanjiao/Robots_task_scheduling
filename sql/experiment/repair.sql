
-- Load backUp tables
TRUNCATE origin_db.charging_stations;
INSERT INTO origin_db.charging_stations(station_id)
VALUES
(18),(19),(20);



-- Make experiment cases
TRUNCATE origin_db.exe_rs;

INSERT INTO origin_db.exe_rs
(wt_btr,wt_wait,wt_psb, wt_pri)
VALUES
(1,1,-1,-25),(1,1,-1,-30),(1,1,-1,-35),(1,1,-1,-40),(1,1,-1,-45),(1,1,-1,-50);

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
