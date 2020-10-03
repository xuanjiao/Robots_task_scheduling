
-- next experiment

DROP PROCEDURE IF EXISTS origin_db.next_env_exp;
DELIMITER ;;

CREATE PROCEDURE origin_db.next_env_exp(
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

END ;;


DELIMITER ;
