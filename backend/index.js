

import express, { json } from 'express';
import cors from 'cors';
import {WebSocketServer} from 'ws';
import routes from './router/nodeRouter.js';
import {createTablesIfNotExist} from './database/database.js';
import mysql from 'mysql';

var corsOptions = {
  origin: "*"
};

// const { WebSocketServer } = require('ws')


const app = express();
app.use(cors(corsOptions));
const sockserver = new WebSocketServer({ port: 443 })

createTablesIfNotExist();


// const upload = multer({ dest: "PDFs/" });
app.use(json());
app.use('/', routes);
app.use(express.static('static'))
  
// set port, listen for requests
const PORT = process.env.PORT || 3000;
app.listen(PORT,'0.0.0.0',() => {
  console.log(`Server is running on port ${PORT}.`);
});

sockserver.on('connection', ws => {
  console.log('New client connected!')
  ws.send('connection established')
  ws.on('close', () => console.log('Client has disconnected!'))
  ws.on('message', data => {
    sockserver.clients.forEach(client => {
      console.log(`distributing message: ${data}`)
      client.send(`${data}`)
    })
  })
  ws.onerror = function () {
    console.log('websocket error')
  }
 })
