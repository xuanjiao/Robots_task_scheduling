
DROP TABLE IF EXISTS exp_db.ese_rs_analysis_1;
CREATE TABLE exp_db.ese_rs_analysis_1
SELECT exp_id,wt_btr, duration/(total-expired), completed/total FROM exp_db.ese_rs_09_16 WHERE task_id IN ();

