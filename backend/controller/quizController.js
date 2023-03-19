import fs from "fs";


export function getQuiz(req, res) {
    var filename = req.query.name
    // get json file data from filename if it exists 
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, PUT, PATCH, DELETE');
    if(filename){
      let rawquizData = fs.readFileSync("uploadedQuiz/"+filename+".json");
      let quizData = JSON.parse(rawquizData);
      res.send(quizData)
    }else {
      res.send("Error: file dosen't exist")}

    }

export function saveQuiz(req, res){


  const quiz = req.body;
  const title = quiz.quiz_name;
  
    fs.writeFile(`uploadedQuiz/${title}.json`,  JSON.stringify(quiz), (err) => {
      if (err) {
        console.error(err);
        return res.status(500).json({ message: `Failed to save Quiz: ${title}` });
      }
  
      res.status(200).json({ message: `Quiz: ${title} saved successfully` });
    });
  
  
  }