
-- next experiment

DROP PROCEDURE IF EXISTS origin_db.next_exe_exp;
DELIMITER ;;

CREATE PROCEDURE origin_db.next_exe_exp(
	IN id_exp INT,
	IN first_task DATETIME,
    OUT go_on BOOLEAN
 )
BEGIN

-- change weight
TRUNCATE origin_db.exe_weight;

SELECT IF(id_exp > MAX(exp_id),false,true) INTO go_on FROM origin_db.exe_rs;
                
INSERT INTO origin_db.exe_weight
SELECT wt_btr,wt_wait,wt_psb, wt_pri FROM origin_db.exe_rs WHERE exp_id = id_exp;

--  Insert some  execute task
SET @task_index := 0;
    
WHILE @task_index < 15 DO
	INSERT INTO origin_db.tasks(task_type, start_time, target_id,priority,dependency)
	VALUES('ExecuteTask',first_task + INTERVAL 30 * (@task_index +1) SECOND,21 + @task_index MOD 10,2,0);
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
