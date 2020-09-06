-- restore
TRUNCATE origin_db.doors;
INSERT INTO origin_db.doors SELECT * FROM back_up_db.doors_bu;

TRUNCATE origin_db.measurements;
INSERT INTO origin_db.measurements SELECT * FROM back_up_db.measurements_bu;

TRUNCATE origin_db.open_possibilities;
INSERT INTO origin_db.open_possibilities SELECT * FROM back_up_db.open_possibilities_bu;

TRUNCATE origin_db.tasks;
INSERT INTO origin_db.tasks SELECT * FROM back_up_db.tasks_bu;

SELECT * FROM origin_db.doors;
SELECT * FROM origin_db.measurements;
SELECT * FROM origin_db.open_possibilities;
SELECT * FROM origin_db.tasks;
