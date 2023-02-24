import mysql from "mysql";

const db = mysql.createConnection({
  host: "localhost",
  user: "root",
  password: "root",
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
    "CREATE TABLE IF NOT EXISTS Results (ResultId int NOT NULL AUTO_INCREMENT, StudentId int NOT NULL, Question_number int NOT NULL, isCorrect bool NOT NULL, Points int NOT NULL, PRIMARY KEY (ResultId));",
    (err, rows, fields) => {
      if (err) throw err;
    }
  );
}

export function insertUser(req, res) {
  var name = req.body.username;
  db.query("INSERT INTO `Students` (`Username`) VALUES ('"+name+"');", (err, rows, fields) => {
    if (err) {
      res.statusMessage = err;
      res.status(500).end();
    } else {
      res.sendStatus(200);
    }
  });
}

export default db;