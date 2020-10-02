

DROP PROCEDURE IF EXISTS create_possibility_table;
delimiter ;;



CREATE PROCEDURE create_possibility_table(IN dayWeek INT, IN door_num INT)
BEGIN
-- DECLARE door INT DEFAULT 1;
SET @door := 1;
SET @zero :=0;
WHILE @door <= door_num
DO
    SET @high :=0.8;
    SET @low  :=0.2;
	SET @possibility_1 := IF(@door%2,@high,@low);
	SET @possibility_2 := IF(@door%2,@low,@high);

    IF @door IN (1,10,17) THEN 
		-- door near corridor
		INSERT IGNORE INTO open_possibilities(door_id,day_of_week,start_time,end_time, open_pos , open_pos_st)
		VALUES
        ( @door,dayWeek,'00:00:00','7:59:59',@high,@high),
		( @door,dayWeek,'08:00:00','8:59:59',@high,@high),
		( @door,dayWeek,'09:00:00','9:59:59',@high,@high),
		( @door,dayWeek,'10:00:00','10:59:59',@high,@high),
		( @door,dayWeek,'11:00:00','11:59:59',@high,@high),
		( @door,dayWeek,'12:00:00','12:59:59',@high,@high),
		( @door,dayWeek,'13:00:00','13:59:59',@high,@high),
		( @door,dayWeek,'14:00:00','14:59:59',@high,@high),
		( @door,dayWeek,'15:00:00','15:59:59',@high,@high),
		( @door,dayWeek,'16:00:00','16:59:59',@high,@high),
        ( @door,dayWeek,'17:00:00','17:59:59',@high,@high),
        ( @door,dayWeek,'18:00:00','18:59:59',@high,@high),
        ( @door,dayWeek,'19:00:00','19:59:59',@high,@high),
        ( @door,dayWeek,'20:00:00','20:59:59',@high,@high),
        ( @door,dayWeek,'21:00:00','21:59:59',@high,@high),
        ( @door,dayWeek,'22:00:00','22:59:59',@high,@high),
        ( @door,dayWeek,'23:00:00','23:59:59',@high,@high);

    ELSE
    
		INSERT IGNORE INTO open_possibilities(door_id,day_of_week,start_time,end_time, open_pos , open_pos_st)
		VALUES
		( @door,dayWeek,'00:00:00','0:59:59',@possibility_1,@possibility_1),
        ( @door,dayWeek,'10:00:00','1:59:59',@possibility_1,@possibility_1),
        ( @door,dayWeek,'02:00:00','2:59:59',@possibility_1,@possibility_1),
        ( @door,dayWeek,'03:00:00','3:59:59',@possibility_1,@possibility_1),
        ( @door,dayWeek,'04:00:00','4:59:59',@possibility_1,@possibility_1),
        ( @door,dayWeek,'05:00:00','5:59:59',@possibility_1,@possibility_1),
        ( @door,dayWeek,'06:00:00','6:59:59',@possibility_1,@possibility_1),
        ( @door,dayWeek,'07:00:00','7:59:59',@possibility_1,@possibility_1),
		( @door,dayWeek,'08:00:00','8:59:59',@possibility_1,@possibility_1),
		( @door,dayWeek,'09:00:00','9:59:59',@possibility_1,@possibility_1),
		( @door,dayWeek,'10:00:00','10:59:59',@possibility_1,@possibility_1),
		( @door,dayWeek,'11:00:00','11:59:59',@possibility_1,@possibility_1),
		( @door,dayWeek,'12:00:00','12:59:59',@possibility_1,@possibility_1),
		( @door,dayWeek,'13:00:00','13:59:59',@possibility_2,@possibility_2),
		( @door,dayWeek,'14:00:00','14:59:59',@possibility_2,@possibility_2),
		( @door,dayWeek,'15:00:00','15:59:59',@possibility_2,@possibility_2),
		( @door,dayWeek,'16:00:00','16:59:59',@possibility_2,@possibility_2),
		( @door,dayWeek,'17:00:00','17:59:59',@possibility_2,@possibility_2),
        ( @door,dayWeek,'18:00:00','18:59:59',@possibility_2,@possibility_2),
        ( @door,dayWeek,'19:00:00','19:59:59',@possibility_2,@possibility_2),
        ( @door,dayWeek,'20:00:00','20:59:59',@possibility_2,@possibility_2),
        ( @door,dayWeek,'21:00:00','21:59:59',@possibility_2,@possibility_2),
        ( @door,dayWeek,'22:00:00','22:59:59',@possibility_2,@possibility_2),
        ( @door,dayWeek,'23:00:00','23:59:59',@possibility_2,@possibility_2);
        
    END IF;
	SET @door = @door + 1;
END WHILE;
END ;;

delimiter ;
-- call create_possibility_table(5,17);