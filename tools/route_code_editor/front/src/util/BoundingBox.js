import * as THREE from 'three';

/** 
 * Create BoundingBox from Multiple Boxes
 * 複数のボックスを覆うボックスを生成する
 */
export default class BoundingBox extends THREE.Box3 {
  
  /**
   * @param {THREE.Vector3} min 
   * @param {THREE.Vector3} max 
   */
  constructor(min, max) {
    super(min, max);
  }
  static createFromBoxes(boxes) {
    let min = {};
    min.x = Math.min.apply(null, boxes.map((box) => {
      return box.min.x; 
    }));
    min.y = Math.min.apply(null, boxes.map((box) => {
      return box.min.y; 
    }));
    min.z = Math.min.apply(null, boxes.map((box) => {
      return box.min.z; 
    }));
    let max = {};
    max.x = Math.max.apply(null, boxes.map((box) => {
      return box.max.x; 
    }));
    max.y = Math.max.apply(null, boxes.map((box) => {
      return box.max.y; 
    }));
    max.z = Math.max.apply(null, boxes.map((box) => {
      return box.max.z; 
    }));
    return new BoundingBox(min, max);
  }
  /**
   * @param {array of THREE.Vector3} points 
   */
  static createFromPoints(points) {
    let min = {};
    min.x = Math.min.apply(null, points.map((point)=>{
      return point.x;
    }));
    min.y = Math.min.apply(null, points.map((point)=>{
      return point.y;
    }));
    min.z = Math.min.apply(null, points.map((point)=>{
      return point.z;
    }));
    let max = {};
    max.x = Math.max.apply(null, points.map((point)=>{
      return point.x;
    }));
    max.y = Math.max.apply(null, points.map((point)=>{
      return point.y;
    }));
    max.z = Math.max.apply(null, points.map((point)=>{
      return point.z;
    }));
    return new BoundingBox(min, max);
  }

}