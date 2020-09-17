
DROP TABLE IF EXISTS exp_db.ese_rs_analysis_1;
CREATE TABLE exp_db.ese_rs_analysis_1
SELECT exp_id,wt_pri,total, completed , completed/total, TIME_TO_SEC(duration)/(total-expired) FROM exp_db.ese_rs_09_16 WHERE exp_id BETWEEN 21 AND 30;
SELECT * FROM exp_db.ese_rs_analysis_1;
