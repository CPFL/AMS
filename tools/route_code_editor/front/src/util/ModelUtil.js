import { Point,  Line, Vector, Pole, Area } from '../model/ThreeJS/adasmap/geometry/index';
import { Lane, Node, Dtlane } from '../model/ThreeJS/adasmap/road/index';
import { Waypoint } from '../model/ThreeJS/adasmap/waypoints/Waypoint';
export default class ModelUtil{

  // 選択オブジェクト(物理モデル)から道路モデルのオブジェクトを取得する
  static lookupRoadObject(object){
    // 選択オブジェクトがPointの時
    if(object instanceof Point){
      let point = object;
      // Node, Dtlaneの時，Node, Dtlaneを返す
      if(point.parent instanceof Node || point.parent instanceof Dtlane){
        return point.parent;
      }
      // PointがAreaの一部の時，AreaBasedObjectを返す
      if(point.parent instanceof Area){
        let area = point.parent;
        return area.parent;
      }
    }
    // 選択オブジェクトがLineの時
    if(object instanceof Line){
      let line = object;
      // LineがAreaの一部の時，AreaBasedObjectを返す
      if(line.parent instanceof Area){
        let area = line.parent;
        return area.parent;
      }else{
        // そうでないとき，LineBasedObjectを返す
        return line.parent;
      }
    }
    // 選択オブジェクトがLaneの時
    if(object instanceof Lane){
      return object;
    }
    if(object instanceof Vector){
      return object.parent;
    }
    if(object instanceof Pole){
      return object.parent;
    }
    if(object instanceof Waypoint){
      return object;
    }
    return null;
  }

}