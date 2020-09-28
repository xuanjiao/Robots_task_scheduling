
DROP TABLE IF EXISTS exp_db.ese_rs_analysis_1;
CREATE TABLE exp_db.ese_rs_analysis_1
SELECT exp_id,wt_btr,total, completed , completed/total, TIME_TO_SEC(duration)/(total-expired) FROM exp_db.exe_rs_09_18 WHERE wt_wait = 1  AND wt_psb = -1 AND wt_pri = -1;
SELECT * FROM exp_db.ese_rs_analysis_1;

-- DROP TABLE IF EXISTS exp_db.exe_rs_09_18;
-- CREATE TABLE exp_db.exe_rs_09_18
-- SELECT * FROM origin_db.exe_rs;
