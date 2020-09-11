DROP PROCEDURE IF EXISTS exp_db.update_exp_result;
DROP PROCEDURE IF EXISTS exp_db.update_next_exp_start_time;
DELIMITER ;;

CREATE PROCEDURE exp_db.update_exp_result()
BEGIN

-- Update current experiment result and finish time
-- exp. finish_time = latest execute task finish time 
UPDATE exp_db.exe_rs rs
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

END ;;

-- next experiment start time = latest charging task finish time
CREATE PROCEDURE exp_db.update_next_exp_start_time()
BEGIN
	UPDATE exp_db.exe_rs rs
	INNER JOIN (
		SELECT 
			FLOOR((t.task_id -1)/ @task_per_exp + 2) AS exp_no, 
			MAX(finish_time) AS st
		FROM origin_db.tasks t WHERE t.task_type = 'Charging'
		GROUP BY exp_no
		) tmp ON rs.exp_no = tmp.exp_no
	SET rs.start_time = tmp.st;

END ;;

DELIMITER ;


-- CALL exp_db.update_exp_result();
-- SELECT * FROM exp_db.exe_rs;