export default class PCDLoader {


  async loadFromLocalFile(fileList) {

    let pcdList = {};
    for (let file of fileList) {
      if (file.name.match('.pcd')) {
        pcdList[file.name] = await this._readFile(file)
      }
    }
    return pcdList
  }

  async _readFile(file) {
    return new Promise((resolve) => {
      let reader = new FileReader();
      reader.readAsArrayBuffer(file);

      reader.addEventListener('load', (e) => {
        resolve(e.target.result);
      })
    });
  }

}