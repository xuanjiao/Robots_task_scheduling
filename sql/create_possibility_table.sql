

DROP PROCEDURE IF EXISTS create_possibility_table;
delimiter ;;



CREATE PROCEDURE create_possibility_table(IN door_num INT)
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
        ( @door,2,'00:00:00','7:59:59',@high,@high),
		( @door,2,'08:00:00','8:59:59',@high,@high),
		( @door,2,'09:00:00','9:59:59',@high,@high),
		( @door,2,'10:00:00','10:59:59',@high,@high),
		( @door,2,'11:00:00','11:59:59',@high,@high),
		( @door,2,'12:00:00','12:59:59',@high,@high),
		( @door,2,'13:00:00','13:59:59',@high,@high),
		( @door,2,'14:00:00','14:59:59',@high,@high),
		( @door,2,'15:00:00','15:59:59',@high,@high),
		( @door,2,'16:00:00','16:59:59',@high,@high),
        ( @door,2,'17:00:00','17:59:59',@high,@high),
        ( @door,2,'18:00:00','18:59:59',@high,@high),
        ( @door,2,'19:00:00','19:59:59',@high,@high),
        ( @door,2,'20:00:00','20:59:59',@high,@high),
        ( @door,2,'21:00:00','21:59:59',@high,@high),
        ( @door,2,'22:00:00','22:59:59',@high,@high),
        ( @door,2,'23:00:00','23:59:59',@high,@high),
		( @door,3,'00:00:00','7:59:59',@high,@high),
		( @door,3,'08:00:00','8:59:59',@high,@high),
		( @door,3,'09:00:00','9:59:59',@high,@high),
		( @door,3,'10:00:00','10:59:59',@high,@high),
		( @door,3,'11:00:00','11:59:59',@high,@high),
		( @door,3,'12:00:00','12:59:59',@high,@high),
		( @door,3,'13:00:00','13:59:59',@high,@high),
		( @door,3,'14:00:00','14:59:59',@high,@high),
		( @door,3,'15:00:00','15:59:59',@high,@high),
		( @door,3,'16:00:00','16:59:59',@high,@high),
        ( @door,3,'17:00:00','17:59:59',@high,@high),
        ( @door,3,'18:00:00','18:59:59',@high,@high),
        ( @door,3,'19:00:00','19:59:59',@high,@high),
        ( @door,3,'20:00:00','20:59:59',@high,@high),
        ( @door,3,'21:00:00','21:59:59',@high,@high),
        ( @door,3,'22:00:00','22:59:59',@high,@high),
        ( @door,3,'23:00:00','23:59:59',@high,@high);

    ELSE
    
		INSERT IGNORE INTO open_possibilities(door_id,day_of_week,start_time,end_time, open_pos , open_pos_st)
		VALUES
		( @door,2,'00:00:00','0:59:59',@possibility_1,@possibility_1),
        ( @door,2,'10:00:00','1:59:59',@possibility_1,@possibility_1),
        ( @door,2,'02:00:00','2:59:59',@possibility_1,@possibility_1),
        ( @door,2,'03:00:00','3:59:59',@possibility_1,@possibility_1),
        ( @door,2,'04:00:00','4:59:59',@possibility_1,@possibility_1),
        ( @door,2,'05:00:00','5:59:59',@possibility_1,@possibility_1),
        ( @door,2,'06:00:00','6:59:59',@possibility_1,@possibility_1),
        ( @door,2,'07:00:00','7:59:59',@possibility_1,@possibility_1),
		( @door,2,'08:00:00','8:59:59',@possibility_1,@possibility_1),
		( @door,2,'09:00:00','9:59:59',@possibility_1,@possibility_1),
		( @door,2,'10:00:00','10:59:59',@possibility_1,@possibility_1),
		( @door,2,'11:00:00','11:59:59',@possibility_1,@possibility_1),
		( @door,2,'12:00:00','12:59:59',@possibility_1,@possibility_1),
		( @door,2,'13:00:00','13:59:59',@possibility_2,@possibility_2),
		( @door,2,'14:00:00','14:59:59',@possibility_2,@possibility_2),
		( @door,2,'15:00:00','15:59:59',@possibility_2,@possibility_2),
		( @door,2,'16:00:00','16:59:59',@possibility_2,@possibility_2),
		( @door,2,'17:00:00','17:59:59',@possibility_2,@possibility_2),
        ( @door,2,'18:00:00','18:59:59',@possibility_2,@possibility_2),
        ( @door,2,'19:00:00','19:59:59',@possibility_2,@possibility_2),
        ( @door,2,'20:00:00','20:59:59',@possibility_2,@possibility_2),
        ( @door,2,'21:00:00','21:59:59',@possibility_2,@possibility_2),
        ( @door,2,'22:00:00','22:59:59',@possibility_2,@possibility_2),
        ( @door,2,'23:00:00','23:59:59',@possibility_2,@possibility_2),
        ( @door,3,'00:00:00','0:59:59',@possibility_1,@possibility_1),
        ( @door,3,'10:00:00','1:59:59',@possibility_1,@possibility_1),
        ( @door,3,'02:00:00','2:59:59',@possibility_1,@possibility_1),
        ( @door,3,'03:00:00','3:59:59',@possibility_1,@possibility_1),
        ( @door,3,'04:00:00','4:59:59',@possibility_1,@possibility_1),
        ( @door,3,'05:00:00','5:59:59',@possibility_1,@possibility_1),
        ( @door,3,'06:00:00','6:59:59',@possibility_1,@possibility_1),
        ( @door,3,'07:00:00','7:59:59',@possibility_1,@possibility_1),
		( @door,3,'08:00:00','8:59:59',@possibility_1,@possibility_1),
		( @door,3,'09:00:00','9:59:59',@possibility_1,@possibility_1),
		( @door,3,'10:00:00','10:59:59',@possibility_1,@possibility_1),
		( @door,3,'11:00:00','11:59:59',@possibility_1,@possibility_1),
		( @door,3,'12:00:00','12:59:59',@possibility_1,@possibility_1),
		( @door,3,'13:00:00','13:59:59',@possibility_2,@possibility_2),
		( @door,3,'14:00:00','14:59:59',@possibility_2,@possibility_2),
		( @door,3,'15:00:00','15:59:59',@possibility_2,@possibility_2),
		( @door,3,'16:00:00','16:59:59',@possibility_2,@possibility_2),
		( @door,3,'17:00:00','17:59:59',@possibility_2,@possibility_2),
        ( @door,3,'18:00:00','18:59:59',@possibility_2,@possibility_2),
        ( @door,3,'19:00:00','19:59:59',@possibility_2,@possibility_2),
        ( @door,3,'20:00:00','20:59:59',@possibility_2,@possibility_2),
        ( @door,3,'21:00:00','21:59:59',@possibility_2,@possibility_2),
        ( @door,3,'22:00:00','22:59:59',@possibility_2,@possibility_2),
        ( @door,3,'23:00:00','23:59:59',@possibility_2,@possibility_2);
    
    END IF;
	SET @door = @door + 1;
END WHILE;
END ;;

delimiter ;