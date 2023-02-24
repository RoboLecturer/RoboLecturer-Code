import { Router } from 'express';
const router = Router();
import { parsePDF,uploadPDF } from '../controller/pdfController.js';
import multer from 'multer'
import path from 'path'
import { getQuiz } from '../controller/quizController.js';
import { insertUser } from '../database/database.js';
import { addResult, resetResults, uploadedFileNames } from '../controller/dbController.js';

const storage = multer.diskStorage({
    destination: function(req, file, cb) {
        if(file.fieldname === 'pdf'){
          cb(null, '../pepperweb/public/uploadedPDFs');

        }else if (file.fieldname === 'quiz'){
          cb(null, 'uploadedQuiz/');

        } else{
          cb(null,false)
        }

    },
    filename: function(req, file, cb) {
        cb(null, file.originalname);
    }
});


// TODO: call parsePDF once uploadPDF is done


var upload = multer({ storage: storage })
const uploadParams = upload.fields([{ name: 'pdf', maxCount: 1 }, { name: 'quiz', maxCount: 1 }])
// upload pdf and quiz to backend
router.post("/upload_pdf",uploadParams,uploadPDF);


// process the pdf (should be called after upload pdf)
router.post('/pdf', parsePDF);
// use filename to get quiz JSON info
router.get('/quiz',getQuiz)

// list the file names of uploaded pdfs
router.get('/uploadedFileNames',uploadedFileNames)


// reset the Results table when starting a new quiz
router.post('/resetResults', resetResults)
// insert a new Student into the DB 
router.post('/insertUser', insertUser)
// add a Student's result to the DB
router.post('/addResult',addResult)
// get get results done winner once done

export default router;