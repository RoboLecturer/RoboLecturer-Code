<template>
  <div>
    <input type="file" ref="pdfInput" @change="uploadPDF"/>
  </div>
</template>

<script>
// temp code for front-end file upload, should get PDF and use POST to call the backend
import axios from 'axios';

export default class Upload extends Vue {
 
    uploadPDF() {
      const pdf = this.$refs.pdfInput.files[0];
      const formData = new FormData();
      formData.append('pdf', pdf);

      axios.post('http://localhost:8001/pdf', formData).then(response => {
        console.log(response.data.textContent);
      });
    }
  
}
</script>
