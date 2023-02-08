import fs from "fs";


export function getQuiz(req, res) {
    var filename = req.query.name
    // get json file data from filename if it exists 
    if(filename){
      let rawquizData = fs.readFileSync("uploadedQuiz/"+filename+".json");
      let quizData = JSON.parse(rawquizData);
        
      res.send(quizData)
    }else {res.send("Error: file dosen't exist")}
}