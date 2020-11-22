-- DROP TABLE IF EXISTS  env_rs;
-- ALTER TABLE origin_db.env_rs MODIFY last_update TIME;

DROP PROCEDURE IF EXISTS update_env_exp_result;
DELIMITER ;;

CREATE PROCEDURE update_env_exp_result()
BEGIN
SET SQL_SAFE_UPDATES = 0;
-- update duration and finish time
UPDATE env_rs 
SET finish_time = ADDTIME(start_time ,'00:20:00'),
duration = '00:20:00' WHERE start_time IS NOT NULL;

SET @id_exp:=1; 
SELECT COUNT(start_time) INTO @exp_cnt FROM origin_db.env_rs;
WHILE ( @id_exp <= @exp_cnt ) DO 

	SELECT start_time, finish_time INTO  @st, @ft 
	FROM origin_db.env_rs
	WHERE exp_id = @id_exp;
    
    -- success or failed task
	UPDATE  origin_db.env_rs
    SET succedded = (SELECT COUNT(task_id) FROM tasks WHERE start_time > @st AND finish_time < @ft AND cur_status = 'Succedded'),
		failed = (SELECT COUNT(task_id) FROM tasks WHERE start_time > @st AND finish_time < @ft AND cur_status <> 'Succedded')
	WHERE exp_id = @id_exp;

	-- last update time
	UPDATE  origin_db.env_rs JOIN
		( 
			SELECT MIN(dl) AS l
			FROM
			(
				SELECT MAX(date_time) AS dl
				FROM origin_db.measurements
				WHERE date_time BETWEEN @st AND @ft AND SECOND(date_time)<>1
				GROUP BY door_id
			)tmp1
		)tmp2 
		
	SET last_update = TIMEDIFF(tmp2.l,@st)
	WHERE exp_id = @id_exp;

	-- average update interval
	UPDATE  origin_db.env_rs JOIN (
	SELECT 
		SEC_TO_TIME(AVG(TIMESTAMPDIFF(SECOND, r2.date_time,r1.date_time))) AS ai
	FROM
		(SELECT 
			(@rownum:=@rownum + 1) AS rownum,
				door_id,
				date_time,
				door_status
		FROM
			origin_db.measurements, (SELECT @rownum:=0) r
		WHERE
			SECOND(date_time) <> 1 AND date_time BETWEEN @st AND @ft 
		ORDER BY door_id , date_time) r1
			LEFT JOIN
		(SELECT 
			(@INDEX:=@INDEX + 1) AS rownum,
				door_id,
				date_time,
				door_status
		FROM
			origin_db.measurements, (SELECT @INDEX:=0) r
		WHERE
			SECOND(date_time) <> 1 AND date_time BETWEEN @st AND @ft 
		ORDER BY door_id , date_time) r2 ON r1.rownum = r2.rownum + 1
			AND r1.door_id = r2.door_id
	)tmp SET avg_interval =  tmp.ai WHERE exp_id = @id_exp;
    SET  @id_exp := @id_exp+1;
END WHILE;

SELECT * FROM env_rs;
END;;

DELIMITER ;


CALL update_env_exp_result();
