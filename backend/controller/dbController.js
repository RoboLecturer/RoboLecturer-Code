import db from '../database/database.js'
import fs from "fs";

export function addResult(req, res)  {
    const { StudentId, Question_number, isCorrect, Points } = req.body;
    console.log(req.body)
    console.log(StudentId, Question_number, isCorrect, Points)
    // Insert the POST data into the "Results" table
    const query = 'INSERT INTO Results (StudentId, Question_number, isCorrect, Points) VALUES (?, ?, ?, ?)';
    db.query(query, [StudentId, Question_number, isCorrect, Points], (err, result) => {
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
        res.status(500).send('rror reseting results.');
        return;
      }
      console.log('Reseted results:', result);
      res.status(200).send('Successfully Reseted results.');
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