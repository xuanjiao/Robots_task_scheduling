
-- DROP TABLE IF EXISTS exp_db.ese_rs_analysis_1;
-- CREATE TABLE exp_db.ese_rs_analysis_1
-- SELECT exp_id,wt_pri,total, completed, duration, completed/total, TIME_TO_SEC(duration)/(total-expired) FROM exp_db.ese_rs_09_16 WHERE exp_id BETWEEN 31 AND 40;
-- SELECT * FROM exp_db.ese_rs_analysis_1;

DROP TABLE IF EXISTS exp_db.exe_rs_09_17;
CREATE TABLE exp_db.exe_rs_09_17
SELECT * FROM origin_db.exe_rs;

DROP TABLE IF EXISTS exp_db.exe_rs_analysis_09_17;
CREATE TABLE exp_db.exe_rs_analysis_09_17
SELECT exp_id,wt_pri,total, completed, duration, completed/total, TIME_TO_SEC(duration)/(total-expired) FROM exp_db.exe_rs_09_17 WHERE exp_id BETWEEN 16 AND 20;
SELECT * FROM exp_db.exe_rs_analysis_09_17;


