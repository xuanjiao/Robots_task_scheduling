
DROP TABLE IF EXISTS exp_db.ese_rs_analysis_1;
CREATE TABLE exp_db.ese_rs_analysis_1
SELECT exp_id,wt_pri,total, completed , completed/total, TIME_TO_SEC(duration)/(total-expired) FROM exp_db.ese_rs_09_16 WHERE exp_id BETWEEN 31 AND 40;
SELECT * FROM exp_db.ese_rs_analysis_1;

DROP TABLE IF EXISTS exp_db.exe_rs_09_22;
CREATE TABLE exp_db.exe_rs_09_22
SELECT * FROM origin_db.exe_rs;