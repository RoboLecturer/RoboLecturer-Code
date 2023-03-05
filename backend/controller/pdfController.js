// @ts-ignore
import fs from "fs";
import { fromPath } from "pdf2pic";
import { imageOCR,connectROS,publishPdfParseTopic,getDirectories } from '../services/pdfService.js';
import path from "path";
import axios from 'axios';

export function parsePDF(req, res) {
  
  const filepath = req.body.filepath
  
  console.log("Parsing PDF file: " + filepath)

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

  convert.bulk(-1).then(() => {

  // get amount of generated pages
  getDirectories(outputDirectory, async function (err, content) {
    
    try {
      const ros = connectROS()
      
      publishNumSlides(pdfName,content.length,ros)
      for (let i = 1; i <content.length+1; i++) {
        // iterate by filename
        const imgName = outputDirectory+"/untitled."+i+".png"
        // ocr
        var text = await imageOCR(imgName)
        // temp demo
        console.log(text)
        // WIP publish to ROS
        publishPdfParseTopic(pdfName,i+1,text,ros)
      } 
    } catch (error) {
      // failed to connect to ROS 
      console.error(error);
    }

    })
  });

}

export function uploadPDF(req, res) {

  res.send({ message: 'Upload successful!' });
  
  const files = JSON.parse(JSON.stringify(req.files));
  const file = files.pdf[0].path
  // run parse pdf
  const payload = {
    filepath: file
  };

  axios.post('http://localhost:3000/pdf', payload)
  .then(response => {
    console.log(response.data);
  })
  .catch(error => {
    console.error(error);
  });

}

