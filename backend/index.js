// var corsOptions = {
//   origin: "http://localhost:8081"
// };

// app.use(cors(corsOptions));

import express, { json } from 'express';
const app = express();
import routes from './router/nodeRouter.js';


// const upload = multer({ dest: "PDFs/" });
app.use(json());
app.use('/', routes);
app.use(express.static('static'))
  
// set port, listen for requests
const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}.`);
});
