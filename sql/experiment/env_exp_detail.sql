SELECT * FROM measurements WHERE date_time BETWEEN '2020-06-01 9:05:22' AND '2020-06-01 9:15:22';
			
SELECT door_id, MAX(date_time) AS dl
FROM origin_db.measurements
WHERE date_time BETWEEN  '2020-06-01 9:05:22' AND'2020-06-01 9:15:22' AND SECOND(date_time)<>1
GROUP BY door_id;

SELECT * 
FROM origin_db.measurements
WHERE date_time BETWEEN  '2020-06-01 9:05:22' AND'2020-06-01 9:15:22' AND SECOND(date_time)<>1;


SELECT * FROM tasks
WHERE start_time >  '2020-06-01 9:05:22' AND finish_time < '2020-06-01 9:15:22'