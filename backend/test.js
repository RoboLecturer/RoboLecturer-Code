import { fromPath } from "pdf2pic";
import { createWorker } from "tesseract.js";
import fs from "fs";
import { time } from "console";
import path from "path";

// const worker = await createWorker();
const pdffilename = 'test/ml.pdf';


// const pdffilename = "ml.pdf"
var pdfName = path.basename(pdffilename)
pdfName = pdfName.substr(0, pdfName.lastIndexOf('.'));
console.log(pdfName)
const outputDirectory = '../splitPDFs/' + pdfName;
fs.mkdirSync(outputDirectory, { recursive: true })

const baseOptions = {
  // powerpoint slide dims
  width: 1280,
  height: 720,
  // av pdf dpi
  density: 330,
  savePath: outputDirectory
};

const convert = fromPath(specimen1, baseOptions);

convert.bulk(-1);

// const dir = './directory';

const dir = '../splitPDFs/test1'
// const fileLength
// async function getSplitLen(dir,cb){
// fs.readdirSync(dir, (err, files) => {
//   console.log(files.length)
// });
// }

function getDirectories(path, callback) {
  fs.readdir(path, function (err, content) {
      if (err) return callback(err)
      callback(null, content)
  })
}

getDirectories(dir, async function (err, content) {
  // content.length

  for (let i = 1; i <content.length+1; i++) {
    const imgName = dir+"/untitled."+i+".png"
    console.log(await imageOCR(imgName))
  } 






})



// console.log(getDirectories(dir))


// console.log(getSplitLen(dir, function (err, content)))

// fs.readdirSync(dir, function(err, content) {
//   if (err) {
//     return err;
//   } else {
//      console.log(content.length)
//   }
// });



// async function ls(path) {
//   const dir = await fs.promises.opendir(path)
//   for await (const dirent of dir) {
//     console.log(dirent.name)
//   }
// }

// // ls(path).then()
// ls('.').catch(console.error)

// const temp ='../splitPDFs/test1/untitled.1.png'
// const temp2 ='../splitPDFs/test1/untitled.2.png'

// // for (let i = 0; i <fileLength; i++) {
// //   console.log(i)
// // } 

// function sleep(ms) {
//   return new Promise((resolve) => {
//     setTimeout(resolve, ms);
//   });
// }
async function imageOCR (imagePath) {
  const worker = await createWorker();  
  await worker.loadLanguage('eng');
  await worker.initialize('eng');
  const { data: { text } } = await worker.recognize(imagePath);
  await worker.terminate();
  return text;
  
}


// // for (let i = 1; i <62+1; i++) {
// //   const imgName = dir+"/untitled."+i+".png"
// //   console.log(await imageOCR(temp))
// //   await sleep(100)
// // } 


// console.log(await imageOCR(temp))
// console.log(await imageOCR(temp2))


  (async () => {
    await worker.loadLanguage('eng');
    await worker.initialize('eng');
    const { data: { text } } = await worker.recognize('output/from-file-to-images/untitled.3.png');
    console.log(text);
    await worker.terminate();
  })();
  pdfParser.loadPDF(req.body.pdf);
  pdfParser.loadPDF(temp);

  pdfParser.on("pdfParser_dataReady", pdfData => {
    const numPages = pdfData.formImage.Pages.length;
    let textContent = [];

    for (let i = 0; i < numPages; i++) {
      const page = pdfData.formImage.Pages[i];
      let pageContent = '';
      page.Texts.forEach(text => {
        pageContent += text.R[0].T;
      });
      textContent.push(pageContent);
    }

    res.send({ textContent });
  });
  const pdfParser = new PDFParser();

  pdfParser.on("pdfParser_dataError", errData => console.error(errData.parserError) );
  // pdfParser.on("readable", meta => console.log("PDF Metadata", meta) );
  // pdfParser.on("data", page => console.log(page ? "One page paged" : "All pages parsed", page));
  pdfParser.on("pdfParser_dataReady", pdfData => {
      fs.writeFile("./test/F1040EZ.fields.json", JSON.stringify(pdfParser.getAllFieldsTypes()), ()=>{console.log("Done.");});
  });

  pdfParser.loadPDF("./test/test1.pdf");