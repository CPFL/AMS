// modelおよびmodelDataの生成ヘルパー

import { Point, PointDataBuilder, Line, LineData,
  VectorData, Vector, PoleData, Pole,  AreaData, Area } from '../model/ThreeJS/adasmap/geometry/index';

export default class ModelCreateHelper {
  
  static createPointFromPosition(adasmap, position){
    let pid = adasmap.incrementMaxId('point');
    let ref = adasmap.ref;
    let pointData = new PointDataBuilder().setPid(pid).setBx(position.y).setLy(position.x).setH(position.z).setRef(ref).build();
    let point = new Point(pointData);
    point.updateRelatedObjects();
    return point;
  }
  // lineと、lineの2点のpointを生成する
  // positionはlineの中心座標。lineの長さは1mでY軸方向
  static createLineFromPosition(adasmap, position){
    let before_point = ModelCreateHelper.createPointFromPosition(adasmap, position.clone().add(new THREE.Vector3(-0.5, 0, 0)));
    let foward_point = ModelCreateHelper.createPointFromPosition(adasmap, position.clone().add(new THREE.Vector3(0.5, 0, 0)));
    let line = ModelCreateHelper.createLine(adasmap, null, null, before_point, foward_point);
    return line;
  }
  static createVectorFromPosition(adasmap, position, hang = 0, vang = 0){
    let point = ModelCreateHelper.createPointFromPosition(adasmap, position);
    let vid = adasmap.incrementMaxId('vector');
    let vector_data = new VectorData(vid, point.data.pid, hang, vang);
    let vector = new Vector(vector_data, point);
    return vector;
  }
  static cloneVector(adasmap, vector){
    return ModelCreateHelper.createVectorFromPosition(adasmap, vector.point.position, vector.data.hang, vector.data.vang);
  }
  static createPoleFromPosition(adasmap, position){
    let vector = ModelCreateHelper.createVectorFromPosition(adasmap, position);
    let plid = adasmap.incrementMaxId('pole');
    let pole_data = new PoleData(plid, vector.data.vid, 2, 0.1);
    let pole = new Pole(pole_data, vector);
    return pole;
  }
  static clonePole(adasmap, pole){
    let vector = ModelCreateHelper.cloneVector(adasmap, pole.vector);
    let plid = adasmap.incrementMaxId('pole');
    let pole_data = new PoleData(plid, vector.data.vid, pole.data.length, pole.data.dim);
    let new_pole = new Pole(pole_data, vector);
    return new_pole;
  }
  static createAreaFromPosition(adasmap, position){
    let point1 = ModelCreateHelper.createPointFromPosition(adasmap, position.clone().add(new THREE.Vector3(-0.5, 0.5, 0)));
    let point2 = ModelCreateHelper.createPointFromPosition(adasmap, position.clone().add(new THREE.Vector3(0.5, 0.5, 0)));
    let point3 = ModelCreateHelper.createPointFromPosition(adasmap, position.clone().add(new THREE.Vector3(0.5, -0.5, 0)));
    let line1 = ModelCreateHelper.createLine(adasmap, null, null, point1, point2);
    let line2 = ModelCreateHelper.createLine(adasmap, line1, null, point2, point3);
    let line3 = ModelCreateHelper.createLine(adasmap, line2, null, point3, point1);
    let aid = adasmap.incrementMaxId('area');
    let area_data = new AreaData(aid, line1.data.lid, line3.data.lid);
    let area = new Area(area_data, [line1, line2, line3]);
    return area;
  }
  /**
   * Lineを作成
   * ※ before_point, fowar_pointがすでに存在する場合
   * before_line、foward_line は nullable
   */
  static createLine(adasmap, before_line, foward_line, before_point, foward_point){
    let lid = adasmap.incrementMaxId('line');
    let blid = before_line ? before_line.data.lid : 0;
    let flid = foward_line ? foward_line.data.lid : 0;
    let bpid = before_point.data.pid;
    let fpid = foward_point.data.pid;
    let lineData = new LineData(lid, bpid, fpid, blid, flid);
    let line = new Line(lineData, before_point, foward_point);
    if(before_line){
      line.setBeforeLine(before_line);
      before_line.setFowardLine(line);
    }
    if(foward_line){
      line.setFowardLine(foward_line);
      foward_line.setBeforeLine(line);
    }
    return line;
  }
  /**
   * 既存のlineを延長する形で新しくLineとそのPointを作成
   * - 既存のLineと、延長する方のPointを引数として渡す
   * Example
   * (a)----(b)----(c)
   * let { line: line_b_c, point: c } = ModelCreateHelper.createLineAlongWithExisting(line_a_b, b);
   *
   * @param {Line} line - 新しく作成するlineに接続するline 
   * @param {Point} point - 接続するlineと新しく作成するlineの接続点
   * @return {{Line, Point}}
   */
  static createLineAlongWithExisting(adasmap, line, point){
    // 新しく作成するlineが既存のlineのblid,flidどちらとして接続されるか判定し、新規作成するpointの座標を計算
    let new_point_pos;
    let new_point = null;
    let before_line_of_new_line = null;
    let foward_line_of_new_line = null;
    let before_point_of_new_line = null;
    let foward_point_of_new_line = null;
    if(line.foward_point == point && line.foward_line == null){
      // 新しく生成するlineは既存のlineにとってのfoward_lineであり
      // 新しく生成するlineにとってのbefore_line
      before_line_of_new_line = line;
      before_point_of_new_line = point;
      let dir = line.foward_point.position.clone().sub(line.before_point.position);
      new_point_pos = line.foward_point.position.clone().add(dir);
      foward_point_of_new_line = new_point = ModelCreateHelper.createPointFromPosition(adasmap, new_point_pos);
    }else if(line.before_point == point && line.before_line == null){
      foward_line_of_new_line = line;
      foward_point_of_new_line = point;
      let dir = line.before_point.position.clone().sub(line.foward_point.position);
      new_point_pos = line.before_point.position.clone().add(dir);
      before_point_of_new_line = new_point = ModelCreateHelper.createPointFromPosition(adasmap, new_point_pos);
    }
    let new_line = ModelCreateHelper.createLine(adasmap, before_line_of_new_line, foward_line_of_new_line, before_point_of_new_line, foward_point_of_new_line);
    return { line: new_line, point: new_point };
  }
  /**
   * 2点のPointの延長線上に新しくPointを作成する
   * 
   * Example
   * (point1)- - - -(point2)- - - -(new_point)
   * @param {ADASMapLoader} adasmap
   * @param {Point} point1 
   * @param {Point} point2 
   * @return {Point} new_point
   */
  static createPointAlongWithExsiting(adasmap, point1, point2){
    let new_point_pos;
    let dir = point2.position.clone().sub(point1.position);
    new_point_pos = point2.position.clone().add(dir);
    return ModelCreateHelper.createPointFromPosition(adasmap, new_point_pos);
  }

}