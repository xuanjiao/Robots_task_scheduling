SET @test1 := '2020-06-01 09:23:56';
SET @test2 := '2020-06-01 09:38:56';
SELECT 
    MAX(date_time) AS dl
FROM
    measurements
WHERE
    date_time BETWEEN @test1 AND  @test2
        AND SECOND(date_time) <> 1
GROUP BY door_id;

SELECT * FROM measurements WHERE date_time BETWEEN  @test1 AND  @test2;

SELECT * FROM tasks WHERE start_time >@test1 AND finish_time <  @test2;

SELECT MAX(date_time)
FROM origin_db.measurements
WHERE date_time BETWEEN @test1 AND  @test2 AND SECOND(date_time)<>1
GROUP BY door_id 