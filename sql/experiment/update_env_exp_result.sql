DROP TABLE IF EXISTS  env_rs_bu;
CREATE TABLE env_rs_bu SELECT * FROM  env_rs;
ALTER TABLE env_rs_bu MODIFY last_update TIME;

DROP PROCEDURE IF EXISTS update_env_exp_result;
DELIMITER ;;

CREATE PROCEDURE update_env_exp_result()
BEGIN
SET SQL_SAFE_UPDATES = 0;
-- update duration and finish time
UPDATE env_rs_bu 
SET finish_time = ADDTIME(start_time ,'00:10:00'),
duration = '00:10:00' WHERE start_time IS NOT NULL;

SET @id_exp:=1; 
SELECT COUNT(start_time) INTO @exp_cnt FROM origin_db.env_rs_bu;
WHILE ( @id_exp <= @exp_cnt ) DO 
	SELECT start_time, finish_time INTO  @st, @ft 
	FROM origin_db.env_rs_bu 
	WHERE exp_id = @id_exp;

	UPDATE  origin_db.env_rs_bu  JOIN
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
		
	SET last_update = tmp2.l
	WHERE exp_id = @id_exp;

	UPDATE  origin_db.env_rs_bu  JOIN (
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
END;;

DELIMITER ;


CALL update_env_exp_result();

SELECT * FROM env_rs_bu;