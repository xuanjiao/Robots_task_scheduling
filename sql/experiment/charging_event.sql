DROP EVENT IF EXISTS origin_db.charging;    -- dynamic charging stations table
DELIMITER ;;

CREATE EVENT origin_db.charging
ON SCHEDULE EVERY 2 SECOND
STARTS CURRENT_TIMESTAMP ENDS CURRENT_TIMESTAMP + INTERVAL 1 HOUR -- Simulation time 

DO BEGIN
     UPDATE origin_db.charging_stations INNER JOIN (
		 SELECT
			SUM(IF(cur_status = 'Charging_finish', 1, 0)) AS sumFinish,
-- 			SUM(IF(cur_status = 'Next_exp', 1, 0)) AS sumNextExp,
-- 			SUM(IF(cur_status = 'Free', 1, 0)) AS sumFree,
			SUM(IF(robot_id IS NOT NULL,1,0)) AS sumExistRobot
			FROM origin_db.charging_stations
     ) tmp
     
     SET cur_status = CASE cur_status WHEN 'Free' 				THEN IF(robot_id is NULL,'Free','Charging')
									  WHEN 'Charging' 			THEN IF(battery >=100,'Charging_finish','Charging') 
                                      WHEN 'Charging_finish' 	THEN IF(tmp.sumFinish = 3, 'Next_exp', 'Charging_finish')
                                      WHEN 'Next_exp'			THEN IF(robot_id is NULL,'Free','Next_exp')
                                      END;
-- 
    --  SELECT SUM(IF(cur_status = 'Charging_finish', 1, 0)) INTO @sumFinish FROM origin_db.charging_stations;
 	-- IF @sumFinish = 3
--      THEN 
-- 		CALL origin_db.next_exp(); 
--  	END IF;
    
	UPDATE origin_db.charging_stations  
      SET battery = CASE cur_status WHEN 'Charging' THEN IF(battery + charging_rate >100,100,battery + charging_rate) END,
		remaining_time = CASE cur_status WHEN 'Charging' THEN (100 - battery)/charging_rate END;
       
	
    
END;;

DELIMITER ;
	-- UPDATE origin_db.charging_stations  
--      SET battery = IF(cur_status = Charging,battery + charging_rate,battery);
--      origin_db.set_charging_status()
--     -- If charging station is not free, 
--     -- battery > = 100 -> Charging_finish
--     -- battery < 100 -> Charging
-- 	UPDATE origin_db.charging_stations 
--     SET battery = IF(battery + charging_rate>=100 AND cur_status <> 'Free' ,100,battery + charging_rate),
-- 	cur_status = IF (battery + charging_rate>=100 AND cur_status <> 'Free' , 'Charging_finish','Charging');
--   
-- 	-- If 3 robot Charging_finish -> Next_exp
-- 	SELECT 
-- 		SUM(IF(cur_status = 'Charging_finish', 1, 0)) ,
-- 		SUM(IF(cur_status = 'Next_exp', 1, 0)) INTO @sumFinish, @sumNextExp
-- 	FROM origin_db.charging_stations;
--     
--     SELECT @sumFinish, @sumNextExp;
--    
--     -- If 3 robot Charging_finish and 
--     IF (@sumFinish = 3 AND @sumNextExp < 3 ) THEN
-- 		UPDATE origin_db.charging_stations	SET cur_status = 'Next_exp';
-- 		CALL origin_db.exp_db.update_exp_result();
-- 		CALL origin_db.exp_db.update_next_exp_start_time();
--     END IF;
--   --   UPDATE origin_db.charging_stations,
-- --     (SELECT 
-- -- 		SUM(IF(cur_status = 'Charging_finish', 1, 0)) as sumRobot
-- --         FROM origin_db.charging_stations
-- --         ) as tmp
-- -- 	SET cur_status = IF(tmp.sumRobot = 0,cur_status,'Next_exp');
--  -- 




-- Old charging event

-- DROP EVENT IF EXISTS origin_db.charging;    -- dynamic charging stations table
-- CREATE EVENT origin_db.charging
-- ON SCHEDULE EVERY 1 SECOND
-- STARTS CURRENT_TIMESTAMP ENDS CURRENT_TIMESTAMP + INTERVAL 3 HOUR
-- DO 
-- 		UPDATE charging_stations 
--         SET robot_battery_level = IF(robot_battery_level + charging_rate>=100,100,robot_battery_level + charging_rate),
-- 			remaining_time =(100 - robot_battery_level)/charging_rate -- Accorging to charging rate, calculate robot_battery_level and charging time 
-- 		WHERE robot_battery_level <100;