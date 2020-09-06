
DROP PROCEDURE IF EXISTS createRawData;
DELIMITER ;;
CREATE PROCEDURE createRawData()
BEGIN
	TRUNCATE TABLE measurements;
	set @tm := '2020-06-01 9:00:01';
    WHILE  @tm < '2020-06-02 20:00:00' DO
    INSERT INTO measurements
    
		SELECT o.door_id,
			IF(rand()< o.open_pos,1,0), 
            @tm 
		FROM open_possibilities o
		WHERE TIME(@tm) BETWEEN o.start_time AND o.end_time AND DAYOFWEEK(@tm) = o.day_of_week;           
		SET @tm := @tm + INTERVAL 20 MINUTE;
    END WHILE;
END ;;
DELIMITER ;

/*
DROP PROCEDURE IF EXISTS createRawData;

delimiter $$
create procedure createRawData()
    in id int,
	in start_date_time datetime,
    in period time,
    in cnt_total int
)

begin    	
	declare cnt int default 0;
    declare cur_time datetime;
    set cur_time = start_date_time;
	while cnt < cnt_total do
          insert into measurements
          select 
            id,
            if(rand()< open_pos,1,0), 
            cur_time -- create door status according to possibility table
                from open_possibilities o
		    where TIME(cur_time) between o.start_time and o.end_time and id = o.door_id and dayofweek(cur_time) = o.day_of_week;
			 -- call process_measurement_result(id,cur_time);
          set cur_time = addtime(cur_time,period);
          set cnt = cnt + 1;
    end while;

    end $$
delimiter ;
*/
