-- Befor experiment, store measurements doors and execute task table

-- DROP DATABASE IF EXISTS back_up_db;
-- CREATE DATABASE back_up_db;
-- 
-- DROP DATABASE IF EXISTS exp_db;
-- CREATE DATABASE exp_db;

DROP TABLE IF EXISTS back_up_db.open_possibilities_bu;
CREATE TABLE back_up_db.open_possibilities_bu
SELECT * FROM origin_db.open_possibilities;

DROP TABLE IF EXISTS back_up_db.charging_stations_bu;
CREATE TABLE back_up_db.charging_stations_bu
SELECT * FROM origin_db.charging_stations;

DROP TABLE IF EXISTS back_up_db.doors_bu;
CREATE TABLE back_up_db.doors_bu
SELECT * FROM origin_db.doors;

DROP TABLE IF EXISTS back_up_db.measurements_bu;
CREATE TABLE back_up_db.measurements_bu
SELECT * FROM origin_db.measurements;

DROP TABLE IF EXISTS back_up_db.tasks_bu;
CREATE TABLE back_up_db.tasks_bu
SELECT * FROM origin_db.tasks;

SELECT * FROM back_up_db.charging_statioins_bu;
SELECT * FROM back_up_db.measurements_bu;
SELECT * FROM back_up_db.doors_bu;
SELECT * FROM back_up_db.open_possibilities_bu;
SELECT * FROM back_up_db.tasks_bu;