DROP PROCEDURE IF EXISTS finish_env_exp;
CREATE PROCEDURE finish_env_exp()
INSERT tasks (task_type,target_id,robot_id,priority)
VALUES
('Charging',18,1,5),
('Charging',19,2,5),
('Charging',20,3,5);

