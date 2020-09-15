DROP PROCEDURE IF EXISTS origin_db.update_exp_result;

DELIMITER ;;

CREATE PROCEDURE origin_db.update_exp_result()
BEGIN

-- Update current experiment result and finish time
-- exp. finish_time = latest execute task finish time 
UPDATE origin_db.exe_rs rs
INNER JOIN (
	SELECT 
 		FLOOR((t.task_id -1)/ @task_per_exp + 1) AS exp_id, 
		COUNT(t.task_id) AS total,
		SUM( IF (t.cur_status = 'RanToCompletion' ,1,0)) AS completed,
		SUM( IF (t.cur_status = 'Created' ,1,0)) AS expired,
		SUM( IF (t.cur_status = 'Canceled' ,1,0)) AS canceled,
		SUM( IF (t.cur_status = 'Error' ,1,0)) AS err, 
		SUM( IF (t.cur_status = 'Running' ,1,0)) AS running,
		SUM( IF (t.cur_status = 'ToReRun' ,1,0)) AS to_rerun,
        MAX(finish_time) AS t
	FROM origin_db.tasks t WHERE t.task_type = 'ExecuteTask'
 	GROUP BY exp_id
	) tmp ON rs.exp_id = tmp.exp_id
SET rs.total = tmp.total, 
	rs.completed = tmp.completed,
	rs.expired = tmp.expired,
	rs.canceled = tmp.canceled,
	rs.err = tmp.err,
	rs.running = tmp.running,
    rs.finish_time = t,
	rs.to_rerun = tmp.to_rerun;

	UPDATE origin_db.exe_rs rs
	INNER JOIN (
		SELECT 
			FLOOR((t.task_id -1)/ @task_per_exp + 1) AS exp_id, 
			MAX(finish_time) AS st
		FROM origin_db.tasks t WHERE t.task_type = 'Charging'
		GROUP BY exp_id
		) tmp ON rs.exp_id = tmp.exp_id
	SET rs.start_time = tmp.st;

 	UPDATE origin_db.exe_rs rs
  SET duration = TIMEDIFF(finish_time,start_time);
    
	SELECT *  FROM origin_db.exe_rs;
END ;;

DELIMITER ;
-- CALL exp_db.update_exp_result();
-- SELECT * FROM exp_db.exe_rs;