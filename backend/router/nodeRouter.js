import { Router } from 'express';
const router = Router();
import { parsePDF,uploadPDF } from '../controller/pdfController.js';
import multer from 'multer'
import path from 'path'
import { getQuiz } from '../controller/quizController.js';
import { insertUser, getUsers } from '../database/database.js';

const storage = multer.diskStorage({
    destination: function(req, file, cb) {
        if(file.fieldname === 'pdf'){
          cb(null, 'uploadedPDFs/');

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

// const storageQuiz = multer.diskStorage({
//   destination: function(req, file, cb) {
//       cb(null, 'uploadedQuiz/');
//   },
//   filename: function(req, file, cb) {
//       cb(null, file.originalname);
//   }
// });


var upload = multer({ storage: storage })
// var quizLoc = multer({ storage: storageQuiz })

router.post('/pdf', parsePDF);
// field name "pdf", example html implementation:
{/* <form action="/upload_pdf" enctype="multipart/form-data" method="post">
  <div class="form-group">
    <input type="file" class="form-control-file" name="pdf">
    <input type="text" class="form-control" placeholder="Number of speakers" name="nspeakers">
    <input type="submit" value="Get me the stats!" class="btn btn-default">
  </div>
</form> */}
const uploadParams = upload.fields([{ name: 'pdf', maxCount: 1 }, { name: 'quiz', maxCount: 1 }])
router.post("/upload_pdf",uploadParams,uploadPDF);

router.get('/quiz',getQuiz)
router.post('/insertUser', insertUser)
router.post('/getUsers', getUsers)

export default router;
