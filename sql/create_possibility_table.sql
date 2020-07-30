DROP PROCEDURE IF EXISTS createPossibilityTable;
delimiter ;;



CREATE PROCEDURE createPossibilityTable(IN door_num INT)
BEGIN
-- DECLARE door INT DEFAULT 1;
SET @door := 1;
SET @zero :=0;
WHILE @door <= door_num
DO
	SET @possibility_1 := IF(@door%2,0.8,0.2);
	SET @possibility_2 := IF(@door%2,0.2,0.8);
    
	INSERT IGNORE INTO open_possibilities(door_id,day_of_week,start_time,end_time, open_pos , open_pos_st)
	VALUES
    ( @door,2,'00:00:00','7:59:59',@zero,@zero),
	( @door,2,'08:00:00','8:59:59',@possibility_1,@possibility_1),
    ( @door,2,'09:00:00','9:59:59',@possibility_1,@possibility_1),
	( @door,2,'10:00:00','10:59:59',@possibility_1,@possibility_1),
	( @door,2,'11:00:00','11:59:59',@possibility_1,@possibility_1),
	( @door,2,'12:00:00','12:59:59',@possibility_1,@possibility_1),
    ( @door,2,'13:00:00','13:59:59',@possibility_2,@possibility_2),
    ( @door,2,'14:00:00','14:59:59',@possibility_2,@possibility_2),
    ( @door,2,'15:00:00','15:59:59',@possibility_2,@possibility_2),
	( @door,2,'16:00:00','16:59:59',@possibility_2,@possibility_2),
	( @door,2,'17:00:00','23:59:59',@zero,@zero);
    
	-- -- -- create raw data with datetime and door status, 
	call createRawData(@door,'2020-06-01 8:00:01','00:20:00',25);
	SET @door = @door + 1;
END WHILE;
END ;;

delimiter ;
