DROP TRIGGER IF EXISTS origin_db.last_task_finsihed;
-- Call this trigger when update task table
delimiter ;;
CREATE TRIGGER origin_db.last_task_finsihed 
AFTER UPDATE ON origin_db.tasks FOR EACH ROW
BEGIN
	-- Split tasks to experiment 1,2,3 ... write experiment result in exec_table
	IF (NEW.cur_status = 'RanToCompletion' AND NEW.start_time = MAX(origin_db.tasks.start_time))  THEN
		UPDATE exp_db.exe_rs rs
		INNER JOIN (
			SELECT 
				FLOOR((t.task_id -1)/ @task_per_exp + 1) AS exp_no, 
				COUNT(t.task_id) AS total,
				SUM( IF (t.cur_status = 'RanToCompletion' ,1,0)) AS completed,
				SUM( IF (t.cur_status = 'Created' ,1,0)) AS not_run,
				SUM( IF (t.cur_status = 'Canceled' ,1,0)) AS canceled,
				SUM( IF (t.cur_status = 'Error' ,1,0)) AS err, 
				SUM( IF (t.cur_status = 'Running' ,1,0)) AS running,
				SUM( IF (t.cur_status = 'ToReRun' ,1,0)) AS to_rerun
			FROM origin_db.tasks t WHERE t.task_type = 'ExecuteTask'
			GROUP BY exp_no
			) tmp ON rs.exp_no = tmp.exp_no
		SET rs.total = tmp.total, 
			rs.completed = tmp.completed,
			rs.not_run = tmp.not_run,
			rs.canceled = tmp.canceled,
			rs.err = tmp.err,
			rs.running = tmp.running,
			rs.to_rerun = tmp.to_rerun;
	END IF;
    
    -- get last task timestamp t, create next round of tests

	CALL exp_db.create_execute_tasks(@task_per_exp,MAX(origin_db.tasks.start_time));
    
    -- update weight table
    INSERT INTO origin_db.exe_weight
		SELECT wt_btr,wt_wait,wt_psb, wt_pri FROM exp_db.exe_rs  WHERE rs.exp_no = COUNT(rs.total)+1;
        
	
END;
;;
DELIMITER ;

    
    