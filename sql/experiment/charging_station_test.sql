call origin_db.set_charging_status();


UPDATE origin_db.charging_stations  
     SET battery = CASE cur_status WHEN 'Charging' THEN IF(battery + charging_rate >100,100,battery + charging_rate) END,
     remaining_time = CASE cur_status WHEN 'Charging' THEN (100 - battery)/charging_rate END;
     
UPDATE origin_db.charging_stations
	SET robot_id = 0, battery = 80 WHERE station_id = 17;
    
UPDATE origin_db.charging_stations
    SET robot_id = 1, battery = 90 WHERE station_id = 18;
    
UPDATE origin_db.charging_stations
    SET robot_id = 2, battery = 93 WHERE station_id = 19;

SELECT * FROM origin_db.charging_stations;

UPDATE origin_db.charging_stations  
	SET robot_id  = NULL WHERE station_id = 17;
    
UPDATE origin_db.charging_stations  
	SET robot_id  = NULL WHERE station_id = 18;
    
UPDATE origin_db.charging_stations  
	SET robot_id  = NULL WHERE station_id = 19;
    