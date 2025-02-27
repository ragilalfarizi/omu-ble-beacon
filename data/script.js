document.getElementById('uploadBtn').addEventListener('click', function() {
    const fileInput = document.getElementById('fileInput');
    const status = document.getElementById('status');

    if (fileInput.files.length === 0) {
        status.textContent = 'Please select a file to upload.';
        status.style.color = 'red';
        return;
    }

    const file = fileInput.files[0];
    const formData = new FormData();
    formData.append('firmware', file);

    fetch('/update', {
        method: 'POST',
        body: formData
    })
    .then(response => response.text())
    .then(data => {
        status.textContent = data;
        status.style.color = 'green';
    })
    .catch(error => {
        status.textContent = 'Upload failed: ' + error;
        status.style.color = 'red';
    });
});
