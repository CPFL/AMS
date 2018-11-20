/**
 * @author Taiga Nishiyama
 * 道路データ - 中心線形データ
 * Road data - Center linear trajectory data
 **/

export class Dtlane extends THREE.Group{

  constructor(dtlaneData, point) {
    super();
    this.name = 'Dtlane';
    this.data = dtlaneData;
    point.addUpdateObject(this);
    point.changeDefaultColor(0xff6868);
    this.point = point;
    this.add(point);
    this.hilight = false;
    this.road = null;
  }

  setRoad(road){
    this.road = road;
  }

  updateRelatedObjects(){
    //
  }
  enableHilight(color){
    this.point.enableHilight(color);
  }
  
  disableHilight(){
    this.point.disableHilight();
  }

  delete(){
    this.road.removeDtlane(this);
  }

}

export class DtlaneData {

  /**
   * 
   * @param {*} did 
   * @param {*} dist - 追加距離(起点0)
   * @param {*} pid
   * @param {*} dir 
   * @param {*} apara 
   * @param {*} r 
   * @param {*} slope 
   * @param {*} cant 
   * @param {*} lw 
   * @param {*} rw 
   */
  constructor(did, dist, pid, dir, apara, r, slope, cant, lw, rw) {
    this.did = parseInt(did, 10);
    this.dist = parseFloat(dist);
    this.pid = parseInt(pid, 10);
    this.dir = parseFloat(dir);
    this.apara = parseFloat(apara);
    this.r = parseFloat(r);
    this.slope = parseFloat(slope);
    this.cant = parseFloat(cant);
    this.lw = parseFloat(lw);
    this.rw = parseFloat(rw);
  }
  toCsv(){
    return this.did + ',' + this.dist + ',' + this.pid + ',' + this.dir + ',' + this.apara + ',' + this.r + ',' + this.slope + ',' + this.cant + ',' + this.lw + ',' + this.rw +'\n';
  }
  getCsvHeader(){
    return 'DID,Dist,PID,Dir,Apara,r,slope,cant,LW,RW\n';
  }

}
