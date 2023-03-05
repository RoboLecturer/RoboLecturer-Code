import db from '../database/database.js'
import fs from "fs";

export function addResult(req, res)  {
    const { StudentId, QuestionNumber, answerIndex, isCorrect, Points } = req.body;
    // Insert the POST data into the "Results" table
    const query = 'INSERT INTO Results (StudentId, QuestionNumber, answerIndex, isCorrect, Points) VALUES (?, ?, ?, ?, ?)';
    db.query(query, [StudentId, QuestionNumber, answerIndex, isCorrect, Points], (err, result) => {
      if (err) {
        console.error('Error adding results: ', err);
        res.status(500).send('Error adding results');
        return;
      }
      console.log('Successfuly added results:', result);
      res.status(200).send('Successfuly added results.');
    });
  };

export function resetResults(req, res) {
    // Delete all rows in the "Results" table
    const query = 'DELETE FROM Results';
    db.query(query, (err, result) => {
      if (err) {
        console.error('Error reseting results: ', err);
        res.status(500).send('Error resetting results.');
        return;
      }
      console.log('Reseted results:', result);
      res.status(200).send('Successfully Reseted results.');
    });
  };

  export function resetLobby(req, res) {
    // Delete all rows in the "Results" table
    const query = 'DELETE FROM Students';
    db.query(query, (err, result) => {
      if (err) {
        console.error('Error reseting students: ', err);
        res.status(500).send('Error resetting students.');
        return;
      }
      console.log('Reset Lobby:', result);
      res.status(200).send('Successfully Reseted lobby.');
    });
  };

export function uploadedFileNames(req, res) {
  const directoryPath = '../pepperweb/public/uploadedPDFs';

  fs.readdir(directoryPath, (err, files) => {
    if (err) {
      return res.status(500).send(err);
    }

    const filenames = files.map(file => file.toString());
    return res.json(filenames);
  });
};

export function insertUser(req, res) {
  var name = req.body.username;
  db.query("INSERT INTO Students (Username) VALUES (?);",[name], (err, rows, fields) => {
    if (err) {
      res.statusMessage = err;
      res.status(500).end();
    } else {
      res.send({studentId: rows.insertId});
    }
  });
}

export function getUsers(req, res) {
    var quiz_id = req.body.quiz_id;
    db.query("SELECT * FROM Students;", (err, rows, fields) => { //Update to select by QuizID
      if (err) {
        res.statusMessage = err;
        res.status(500).end();
      } else {
        res.send(rows);
      }
    });
  }

  export function getResults(req, res) {
    var qid = req.query.question_id
    db.query("SELECT answerIndex, count(answerIndex) as count FROM Results WHERE QuestionNumber = ? GROUP BY answerIndex;",[parseInt(qid)], (err, rows, fields) => { //Update to select by QuizID
      if (err) {
        res.statusMessage = err;
        res.status(500).end();
      } else {
        console.log(rows)
        res.send(rows);
      }
    });
  }

  export function getLeaderboard(req, res) {
    db.query("select StudentId, Username, sum(isCorrect*100) as total_points from Results inner join Students using(StudentId) group by StudentId order by total_points desc;", (err, rows, fields) => { //Update to select by QuizID
      if (err) {
        res.statusMessage = err;
        res.status(500).end();
      } else {
        res.send(rows);
      }
    });
  }


  