/**
 * @author Taiga Nishiyama
 * Signaldata - 信号
 **/

export class Signaldata extends THREE.Group {
  
  constructor(signaldata_data, vector){
    super();
    this.name = 'Signaldata';
    this.data = signaldata_data;
    this.vector = vector;
    this.add(vector);
    this.hilight = false;
    this.road_structure = null;
    this.pole = null;
    this.setType(signaldata_data.type);
  }
  setPole(pole){
    this.pole = pole;
  }
  setRoadStructure(road_structure){
    this.road_structure = road_structure;
  }
  /**
   * 
   * @param {number} type 
   */
  setType(type){
    let color;
    if(type === 1 || type === 4){
      color = 0xff9191; // Red
    }else if(type === 2 || type === 5){
      color = 0x9591ff; // Blue
    }else if(type === 3){
      color = 0xfffcb2; // Yellow
    }else if(type === 9){
      color = 0xffaa00;
    }else{
      console.error('Signaldata setType error');
      console.error('type: ' + type);
    }
    this.data.type = type;
    this.vector.changeDefaultColor(color);
  }

  getSelectableObjects(){
    return [this.vector];
  }

  clone(){
    return this.road_structure.cloneVectorBasedObject(this);
  }

  delete(){
    this.parent.remove(this);
  }

}

export class SignaldataData {
  
  constructor(id, vid, plid, type, link_id){
    this.id = parseInt(id, 10);
    this.vid = parseInt(vid, 10);
    this.plid = parseInt(plid, 10);
    this.type = parseInt(type, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.vid + ',' + this.plid + ',' + this.type + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,VID,PLID,Type,LinkID\n';
  }

}
