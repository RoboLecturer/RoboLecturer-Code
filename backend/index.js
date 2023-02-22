

import express, { json } from 'express';
import cors from 'cors';
import routes from './router/nodeRouter.js';
import {createTablesIfNotExist} from './database/database.js';
import mysql from 'mysql';

var corsOptions = {
  origin: "http://localhost:8080"
};

const app = express();
app.use(cors(corsOptions));


createTablesIfNotExist();


// const upload = multer({ dest: "PDFs/" });
app.use(json());
app.use('/', routes);
app.use(express.static('static'))
  
// set port, listen for requests
const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}.`);
});
