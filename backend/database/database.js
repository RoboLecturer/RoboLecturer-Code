import mysql from "mysql";

const db = mysql.createConnection({
  host: "localhost",
  user: "adminuser",
  password: "admin",
  database: "roboserver",
});

// Connect to MySQL database
db.connect((err) => {
  if (err) {
    console.error('Error connecting to MySQL database: ', err);
    throw err;
  }
  console.log('Connected to MySQL roboserver database.');
});


export function createTablesIfNotExist() {
  db.query(
    "CREATE TABLE IF NOT EXISTS Students (StudentId int NOT NULL AUTO_INCREMENT, Username varchar(255) NOT NULL, PRIMARY KEY (StudentId));",
    (err, rows, fields) => {
      if (err) throw err;
    }
  );
  db.query(
    "CREATE TABLE IF NOT EXISTS Results (ResultId int NOT NULL AUTO_INCREMENT, StudentId int NOT NULL, QuestionNumber int NOT NULL, answerIndex int NOT NULL, isCorrect bool NOT NULL, Points int NOT NULL, PRIMARY KEY (ResultId), UNIQUE KEY student_result_unique_key (StudentId, QuestionNumber));",
    (err, rows, fields) => {
      if (err) throw err;
    }
  );
}


export default db;
