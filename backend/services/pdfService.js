import fs from "fs";
import ROSLIB from "roslib";
import { createWorker } from "tesseract.js";



export function connectROS(){
var ros = new ROSLIB.Ros({
url : 'ws://localhost:9000'
});

ros.on('connection', function() {
console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
console.log('Connection to websocket server closed.');
});
return ros
}

export function publishPdfParseTopic(filename,pageNumber,text,ros ){
var publishTopic = new ROSLIB.Topic({
ros : ros,
name : '/slides',
messageType : 'std_msgs/String'
});

var msg = new ROSLIB.Message({
metadata : {
fileName : filename,
pageNumber : pageNumber
},
data : {
text
}
});
publishTopic.publish(msg);
}

export function publishNumSlides(filename,pagecount,ros ){
    var publishTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/num_slides',
    messageType : 'std_msgs/String'
    });
    
    var msg = new ROSLIB.Message({
    FormDataEvent : {
    fileName : filename,
    pageCount : pagecount
    }
    });
    publishTopic.publish(msg);
    }

export async function imageOCR (imagePath) {
    const worker = await createWorker();  
    await worker.loadLanguage('eng');
    await worker.initialize('eng');
    const { data: { text } } = await worker.recognize(imagePath);
    await worker.terminate();
    return text;
    
}

export function getDirectories(path, callback) {
fs.readdir(path, function (err, content) {
    if (err) return callback(err)
    callback(null, content)
})
}