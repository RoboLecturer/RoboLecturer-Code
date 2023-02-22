import mysql from "mysql";

const connection = mysql.createConnection({
  host: "localhost",
  user: "adminuser",
  password: "admin",
  database: "roboserver",
});

connection.connect();

export function createTablesIfNotExist() {
  connection.query(
    "CREATE TABLE IF NOT EXISTS Students (StudentId int NOT NULL AUTO_INCREMENT, Username varchar(255) NOT NULL, PRIMARY KEY (StudentId));",
    (err, rows, fields) => {
      if (err) throw err;
    }
  );
  connection.query(
    "CREATE TABLE IF NOT EXISTS Results (ResultId int NOT NULL AUTO_INCREMENT, StudentId int NOT NULL, QuizId int NOT NULL, Points int NOT NULL, PRIMARY KEY (ResultId));",
    (err, rows, fields) => {
      if (err) throw err;
    }
  );
}

export function insertUser(req, res) {
  var name = req.body.username;
  connection.query("INSERT INTO `Students` (`Username`) VALUES ('"+name+"');", (err, rows, fields) => {
    if (err) {
      res.statusMessage = err;
      res.status(500).end();
    } else {
      res.sendStatus(200);
    }
  });
}
