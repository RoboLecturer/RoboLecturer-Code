// @ts-ignore
import fs from "fs";
import { fromPath } from "pdf2pic";
import { imageOCR,connectROS,publishPdfParseTopic,getDirectories } from '../services/pdfService.js';
import path from "path";


export function parsePDF(req, res) {
  
  const filepath = req.body.filepath
  console.log(filepath)


  // check if file exists
  if (!fs.existsSync(filepath)) {
    res.json({message: "Error, file does not exist" }) 
  }

  // extract filename to create new dir
  var pdfName = path.basename(filepath)
  pdfName = pdfName.substr(0, pdfName.lastIndexOf('.'));
  const outputDirectory = 'splitPDFs/' + pdfName;
  fs.mkdirSync(outputDirectory, { recursive: true })

  const baseOptions = {
    // powerpoint slide dims
    width: 1280,
    height: 720,
    // av pdf dpi
    density: 330,
    savePath: outputDirectory
  };

  // create images in dir from pdf
  const convert = fromPath(filepath, baseOptions);

  convert.bulk(-1);


  // get amount of generated pages
  getDirectories(outputDirectory, async function (err, content) {
    

    try {
      const ros = connectROS()
    
      for (let i = 1; i <content.length+1; i++) {
        // iterate by filename
        const imgName = outputDirectory+"/untitled."+i+".png"
        // ocr
        var text = await imageOCR(imgName)
        // temp demo
        console.log(text)
        // WIP public to ROS
        publishPdfParseTopic(pdfName,i+1,text,ros)
      } 
    } catch (error) {
      // failed to connect to ROS 
      console.error(error);
    }

    })


}

export function uploadPDF(req, res) {
    console.log(req.file);
    res.json({ message: "Successfully uploaded files" });
}

