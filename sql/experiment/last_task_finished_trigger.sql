DROP TRIGGER IF EXISTS origin_db.last_task_finsihed;
DROP TRIGGER IF EXISTS origin_db.new_weight;
-- Call this trigger when update task tablea
delimiter ;;
																																										
CREATE TRIGGER origin_db.last_task_finsihed 
AFTER UPDATE ON origin_db.tasks FOR EACH ROW
BEGIN
	
	-- Split tasks to experiment 1,2,3 ... write experiment result in exec_task result 
	IF NEW.cur_status IN ('RanToCompletion','Error', 'Canceled') AND  NEW.task_id = @LAST_TASK  THEN
		CALL exp_db.update_exp_result();
    
  END IF;

    
END;;


CREATE TRIGGER robot
AFTER UPDATE ON exp_db.exe_rs FOR EACH ROW
BEGIN
      -- get last task timestamp t, create next round of tests
          -- update weight table
    DELETE FROM origin_db.exe_weight;
    INSERT INTO origin_db.exe_weight
		SELECT wt_btr,wt_wait,wt_psb, wt_pri FROM exp_db.exe_rs  WHERE exp_no = (SELECT COUNT(total) +1 FROM exp_db.exe_rs);
																	
	
    
	CALL origin_db.create_execute_tasks(@task_per_exp,@LAST_TIME,@LAST_TASK);
    
       
END;;

DELIMITER ;