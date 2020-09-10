DROP PROCEDURE IF EXISTS exp_db.update_exp_result;
DELIMITER ;;

CREATE PROCEDURE exp_db.update_exp_result()
BEGIN
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
	FROM exp_db.execute_tasks t WHERE t.task_type = 'ExecuteTask'
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
DELIMITER ;

CALL exp_db.update_exp_result();
SELECT * FROM exp_db.exe_rs;