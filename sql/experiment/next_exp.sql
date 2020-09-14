
-- next experiment

DROP PROCEDURE IF EXISTS origin_db.next_exp;
DELIMITER ;;

CREATE PROCEDURE origin_db.next_exp(
	IN id_exp INT,
	IN first_task DATETIME
 )
BEGIN

-- change weight
TRUNCATE origin_db.exe_weight;

INSERT INTO origin_db.exe_weight
SELECT wt_btr,wt_wait,wt_psb, wt_pri FROM origin_db.exe_rs WHERE exp_id = id_exp;

--  Insert some  execute task
SET @task_index := 1;
    
WHILE @task_index <= 15 DO
	INSERT INTO origin_db.tasks(task_type, start_time, target_id,priority,dependency)
	VALUES('ExecuteTask',first_task + INTERVAL 20 * @task_index SECOND,21 + @task_index MOD 9,2,0);
	SET @task_index := @task_index +1;
END WHILE;

-- Insert some charging task
INSERT tasks (task_type,target_id,robot_id,priority)
VALUES
('Charging',18,1,5),
('Charging',19,2,5),
('Charging',20,3,5);

END ;;


DELIMITER ;

/*
UPDATE origin_db.exe_rs rs
INNER JOIN (
	SELECT 
 		FLOOR((t.task_id -1)/ @task_per_exp + 1) AS exp_no, 
		COUNT(t.task_id) AS total,
		SUM( IF (t.cur_status = 'RanToCompletion' ,1,0)) AS completed,
		SUM( IF (t.cur_status = 'Created' ,1,0)) AS expired,
		SUM( IF (t.cur_status = 'Canceled' ,1,0)) AS canceled,
		SUM( IF (t.cur_status = 'Error' ,1,0)) AS err, 
		SUM( IF (t.cur_status = 'Running' ,1,0)) AS running,
		SUM( IF (t.cur_status = 'ToReRun' ,1,0)) AS to_rerun,
        MAX(finish_time) AS t
	FROM origin_db.tasks t WHERE t.task_type = 'ExecuteTask'
 	GROUP BY exp_no
	) tmp ON rs.exp_no = tmp.exp_no
SET rs.total = tmp.total, 
	rs.completed = tmp.completed,
	rs.expired = tmp.expired,
	rs.canceled = tmp.canceled,
	rs.err = tmp.err,
	rs.running = tmp.running,
    rs.finish_time = t,
	rs.to_rerun = tmp.to_rerun;
    

-- next experiment start time = latest charging task finish time
UPDATE origin_db.exe_rs rs
INNER JOIN (
	SELECT 
		FLOOR((t.task_id -1)/ @task_per_exp + 1) AS exp_no, 
		MAX(finish_time) AS st 
	FROM origin_db.tasks t WHERE t.task_type = 'Charging'
	GROUP BY exp_no
	) tmp ON rs.exp_no = tmp.exp_no
SET rs.start_time = tmp.st;

*/