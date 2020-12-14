
-- next experiment

DROP PROCEDURE IF EXISTS origin_db.next_env_exp;
DELIMITER ;;

CREATE PROCEDURE origin_db.next_env_exp(
	IN id_exp INT,
    IN t DATETIME,
    OUT go_on BOOLEAN
 )
BEGIN

-- change weight
TRUNCATE origin_db.door_weight;

SELECT IF(id_exp > MAX(exp_id),false,true) INTO go_on FROM origin_db.env_rs;
                
INSERT INTO origin_db.door_weight
SELECT wt_btr, wt_update, wt_psb FROM origin_db.env_rs WHERE exp_id = id_exp;

-- SELECT MIN(last_update) INTO @up FROM doors;
    
UPDATE origin_db.env_rs 
SET start_time = t
WHERE exp_id = id_exp;

UPDATE doors SET last_update = t;
END ;;


DELIMITER ;
