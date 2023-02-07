import { Router } from 'express';
const router = Router();
import { parsePDF,uploadPDF } from '../controller/pdfController.js';
import multer from 'multer'
import path from 'path'

const storage = multer.diskStorage({
    destination: function(req, file, cb) {
        cb(null, 'uploadedPDFs/');
    },
    filename: function(req, file, cb) {
        cb(null, file.originalname);
    }
});

var upload = multer({ storage: storage })

router.post('/pdf', parsePDF);
// field name "pdf", example html implementation:
{/* <form action="/upload_pdf" enctype="multipart/form-data" method="post">
  <div class="form-group">
    <input type="file" class="form-control-file" name="pdf">
    <input type="text" class="form-control" placeholder="Number of speakers" name="nspeakers">
    <input type="submit" value="Get me the stats!" class="btn btn-default">
  </div>
</form> */}
router.post("/upload_pdf", upload.single("pdf"),uploadPDF);

export default router;
