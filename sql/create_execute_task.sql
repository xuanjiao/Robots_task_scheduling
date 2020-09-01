-- Create some execute task
-- points 19-28  
DROP PROCEDURE IF EXISTS create_execute_tasks;

delimiter ;;

CREATE PROCEDURE create_execute_tasks()
BEGIN
 SET @task_type := 'ExecuteTask';
 SET @start_time := '2020-06-01 9:00:20';
 SET @task_num := 20;
 SET @task_index := 0;
 WHILE @task_index < @task_num DO
	-- Create task to a random point. priority 2, dependency 0
	INSERT INTO tasks(task_type,start_time, target_id, priority,dependency)
		VALUES(@task_type, @start_time,FLOOR(19 + RAND()*10),2,0);
	SET @task_index := @task_index + 1;
    SET @start_time := @start_time + INTERVAL FLOOR(5 * RAND()) MINUTE;
 END WHILE ;
 -- Make some task dependent on a task before
 UPDATE tasks SET dependency = task_id - 1 WHERE task_id MOD 10 = 2;

END ;;

delimiter ;

TRUNCATE tasks;
CALL create_execute_tasks();
SELECT * FROM tasks;
/*
INSERT INTO tasks
VALUES
-- -- task_id, task_type, start_time, target_id, robot_id, priority, cur_status, dependency, result
(1, 'ExecuteTask', '2020-06-01 9:00:20', 19, NULL, 3, 'Created', 0, NULL),
(2, 'ExecuteTask', '2020-06-01 9:00:30', 22, NULL, 3, 'Created', 0, NULL),
(3, 'ExecuteTask', '2020-06-01 9:00:40', 25, NULL, 3, 'Created', 0, NULL),
(4, 'ExecuteTask', '2020-06-01 9:00:30', 24, NULL, 3, 'Created', 1, NULL),
(5, 'ExecuteTask', '2020-06-01 9:01:00', 20, NULL, 2, 'Created', 0, NULL),
(6, 'ExecuteTask', '2020-06-01 9:01:30', 19, NULL, 2, 'Created', 3, NULL),
(7, 'ExecuteTask', '2020-06-01 9:00:20', 20, NULL, 3, 'Created', 0, NULL),
(8, 'ExecuteTask', '2020-06-01 9:00:30', 21, NULL, 3, 'Created', 0, NULL),
(9, 'ExecuteTask', '2020-06-01 9:00:40', 5, NULL, 3, 'Created', 0, NULL),
(10, 'ExecuteTask', '2020-06-01 9:00:30', 21, NULL, 3, 'Created', 1, NULL),
(11, 'ExecuteTask', '2020-06-01 9:01:00', 22, NULL, 2, 'Created', 0, NULL),
(12, 'ExecuteTask', '2020-06-01 9:01:30', 23, NULL, 2, 'Created', 3, NULL),
(13, 'ExecuteTask', '2020-06-01 9:00:20', 20, NULL, 3, 'Created', 0, NULL),
(14, 'ExecuteTask', '2020-06-01 9:00:30', 1, NULL, 3, 'Created', 0, NULL),
(15, 'ExecuteTask', '2020-06-01 9:00:40', 5, NULL, 3, 'Created', 0, NULL),
(16, 'ExecuteTask', '2020-06-01 9:00:30', 21, NULL, 3, 'Created', 1, NULL),
(17, 'ExecuteTask', '2020-06-01 9:01:00', 22, NULL, 2, 'Created', 0, NULL),
(18, 'ExecuteTask', '2020-06-01 9:01:30', 23, NULL, 2, 'Created', 3, NULL),
(19, 'ExecuteTask', '2020-06-01 9:00:20', 20, NULL, 3, 'Created', 0, NULL),
(20, 'ExecuteTask', '2020-06-01 9:00:30', 1, NULL, 3, 'Created', 0, NULL),
(21, 'ExecuteTask', '2020-06-01 9:00:40', 5, NULL, 3, 'Created', 0, NULL),
(22, 'ExecuteTask', '2020-06-01 9:00:30', 21, NULL, 3, 'Created', 1, NULL),
(23, 'ExecuteTask', '2020-06-01 9:01:00', 22, NULL, 2, 'Created', 0, NULL),
(24, 'ExecuteTask', '2020-06-01 9:01:30', 23, NULL, 2, 'Created', 3, NULL),
(25, 'ExecuteTask', '2020-06-01 9:00:20', 20, NULL, 3, 'Created', 0, NULL),
(26, 'ExecuteTask', '2020-06-01 9:00:30', 1, NULL, 3, 'Created', 0, NULL),
(27, 'ExecuteTask', '2020-06-01 9:00:40', 5, NULL, 3, 'Created', 0, NULL),
(28, 'ExecuteTask', '2020-06-01 9:00:30', 21, NULL, 3, 'Created', 1, NULL),
(29, 'ExecuteTask', '2020-06-01 9:01:00', 22, NULL, 2, 'Created', 0, NULL),
(30, 'ExecuteTask', '2020-06-01 9:01:30', 23, NULL, 2, 'Created', 3, NULL);

*/


