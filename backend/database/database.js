
export function createTablesIfNotExist(connection){
    connection.query('CREATE TABLE IF NOT EXISTS Students (StudentId int NOT NULL AUTO_INCREMENT, Username varchar(255) NOT NULL, PRIMARY KEY (StudentId));', (err, rows, fields)=>{
        if (err) throw err;
    })
    connection.query('CREATE TABLE IF NOT EXISTS Results (ResultId int NOT NULL AUTO_INCREMENT, StudentId int NOT NULL, QuizId int NOT NULL, Points int NOT NULL, PRIMARY KEY (ResultId));', (err, rows, fields)=>{
        if (err) throw err;
    })
}