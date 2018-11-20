import proj4 from 'proj4';

/**
 * Converter of Japan Plane Rectangular CS and Lat/Lon, Calculator of meshcode from Lat/Lon.
 * 
 * ADAS-MAP(v.1.20) use JGD2011
 * EPSGCode of JGD2011
 * - Lat/Lon: EPSG:6668
 * - Japan Plane Rectangular CS  I~XIX: EPSG6669 - EPSG:6687
 * 
 * 平面直角座標系と緯度経度の相互変換、緯度経度からメッシュコードを計算するUtility
 * ADAS-MAP(v.1.20) では測地系として日本測地系2011を使います
 * 日本測地系2011(JGD2011)のEPSGコード
 * - 緯度経度: EPSG:6668
 * - 平面直角座標系1~19: EPSG6669 - EPSG:6687
 * 
 * Ref: https://github.com/proj4js/proj4js
 * Ref: http://spatialreference.org/ref/epsg/2443/
 */
class LocationUtil {

  constructor() {
    this.proj4 = proj4;
    this.proj4.defs([
      // JGD2000 LatLon
      ['EPSG:4612', '+proj=longlat +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +no_defs'],
      // JGD2000 Japan Plane Rectangular CS  I~XIX
      ['EPSG:2443', '+proj=tmerc +lat_0=33 +lon_0=129.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2444', '+proj=tmerc +lat_0=33 +lon_0=131 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2445', '+proj=tmerc +lat_0=36 +lon_0=132.1666666666667 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2446', '+proj=tmerc +lat_0=33 +lon_0=133.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2447', '+proj=tmerc +lat_0=36 +lon_0=134.3333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2448', '+proj=tmerc +lat_0=36 +lon_0=136 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2449', '+proj=tmerc +lat_0=36 +lon_0=137.1666666666667 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2450', '+proj=tmerc +lat_0=36 +lon_0=138.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2451', '+proj=tmerc +lat_0=36 +lon_0=139.8333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2452', '+proj=tmerc +lat_0=40 +lon_0=140.8333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2453', '+proj=tmerc +lat_0=44 +lon_0=140.25 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2454', '+proj=tmerc +lat_0=44 +lon_0=142.25 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2455', '+proj=tmerc +lat_0=44 +lon_0=144.25 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2456', '+proj=tmerc +lat_0=26 +lon_0=142 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2457', '+proj=tmerc +lat_0=26 +lon_0=127.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2458', '+proj=tmerc +lat_0=26 +lon_0=124 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2459', '+proj=tmerc +lat_0=26 +lon_0=131 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2460', '+proj=tmerc +lat_0=20 +lon_0=136 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      ['EPSG:2461', '+proj=tmerc +lat_0=26 +lon_0=154 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs'],
      // JGD2011 Latlon
      ['EPSG:6668', '+proj=longlat +ellps=GRS80 +no_defs'],
      // JGD2011 Japan Plane Rectangular CS  I~XIX
      ['EPSG:6669', '+proj=tmerc +lat_0=33 +lon_0=129.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6670', '+proj=tmerc +lat_0=33 +lon_0=131 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6671', '+proj=tmerc +lat_0=36 +lon_0=132.1666666666667 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6672', '+proj=tmerc +lat_0=33 +lon_0=133.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6673', '+proj=tmerc +lat_0=36 +lon_0=134.3333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6674', '+proj=tmerc +lat_0=36 +lon_0=136 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6675', '+proj=tmerc +lat_0=36 +lon_0=137.1666666666667 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6676', '+proj=tmerc +lat_0=36 +lon_0=138.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6677', '+proj=tmerc +lat_0=36 +lon_0=139.8333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6678', '+proj=tmerc +lat_0=40 +lon_0=140.8333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6679', '+proj=tmerc +lat_0=44 +lon_0=140.25 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6680', '+proj=tmerc +lat_0=44 +lon_0=142.25 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6681', '+proj=tmerc +lat_0=44 +lon_0=144.25 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6682', '+proj=tmerc +lat_0=26 +lon_0=142 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6683', '+proj=tmerc +lat_0=26 +lon_0=127.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6684', '+proj=tmerc +lat_0=26 +lon_0=124 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6685', '+proj=tmerc +lat_0=26 +lon_0=131 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6686', '+proj=tmerc +lat_0=20 +lon_0=136 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
      ['EPSG:6687', '+proj=tmerc +lat_0=26 +lon_0=154 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs'],
    ]);
    this.latLonProjectionName = 'JGD2011';
  }
  changeLatlonProjection(name){
    this.latLonProjectionName = name;
  }
  /**
   * B(Lat) + L(Lon) + Ref -> Bx, Ly
   * @param {number} b(float) 10進数緯度
   * @param {number} l(float) 10進数経度
   * @param {number} ref(int)
   */
  toXY(b, l, ref){
    let sourcePoint = new proj4.toPoint([l, b]);
    let sourceProjection = this._getLatlonProjection();
    let destProjection = this._getPlaneRectangularProjection(ref);
    let destPoint = proj4.transform(sourceProjection, destProjection, sourcePoint);
    return { bx: destPoint.y, ly: destPoint.x };
  }
  /**
   * Bx, Ly, Ref -> B(Lat) + L(Lon)
   * @param {number} bx(float)
   * @param {number} ly(float)
   * @param {number} ref(int)
   * @return {{b: number, l: number}} - b: Latitude, l: longitude
   */
  toLatLon(bx, ly, ref){
    let sourcePoint = new proj4.toPoint([ly, bx]);
    let sourceProjection = this._getPlaneRectangularProjection(ref);
    let destProjection = this._getLatlonProjection();
    let destPoint = proj4.transform(sourceProjection, destProjection, sourcePoint);
    return { b: destPoint.y, l: destPoint.x };
  }
  /**
   * B(Lat) + L(Lon) -> MCODE1, MCODE2, MCODE3
   * Ref: https://qiita.com/mormor/items/3643847653e21007f29d#comment-71620425b46ff921c007
   * @param {number} b(float) 10進数緯度
   * @param {number} l(float) 10進数経度
   * @return {{mcode1: int, mcode2: int, mcode3: int}}
   */
  toMeshcode(b, l){
    let latitude = b;
    let longtude = l;
    let p = Math.floor(latitude * 60 / 40);
    let a = latitude * 60 - p * 40;
    let u = Math.floor(longtude - 100);
    let f = longtude - (u + 100);
    let q = Math.floor(a / 5);
    let _b = a - q * 5;
    let v = Math.floor(f * 60 / 7.5);
    let g = f * 60 - v * 7.5;
    let r = Math.floor(_b * 60 / 30);
    //let c = b * 60 - r * 30;
    let w = Math.floor(g * 60 / 45);
    //let h = g * 60 - w * 45;
    let mcode1 = parseInt(String(p).padStart(2, '0') + String(u).padStart(2, '0'), 10);
    let mcode2 = parseInt(q + '' + v, 10);
    let mcode3 = parseInt(r + '' + w, 10);
    return { mcode1, mcode2, mcode3 };
  }
  /**
   * DEG(10進数、度表記) -> DMS(60進数、度分秒表記)
   */
  toDMS(degree){
    // Format: D.MM.SS
    // get 'D'
    let deg = degree > 0 ? Math.floor(degree) : Math.ceil(degree);
    // get 'MM'
    // 小数点以下を取得(degreeを文字列にした後、'.'で分割し、ピリオド以降(小数点以下)を取得。数値化)
    let deciam_val = parseFloat('0.' + String(degree).split('.')[1]);
    // 60を掛けた整数部分が分
    let mm = Math.floor(deciam_val * 60);
    // get 'SS'
    // 小数点以下を取得
    deciam_val = parseFloat('0.' + String(deciam_val * 60).split('.')[1]);
    // 60を掛けた結果が秒
    let ss = deciam_val * 60;
    let ss_int = String(ss).split('.')[0];
    let ss_decimal = String(ss).split('.')[1];
    let dms_str = deg + '.' + String(mm).padStart(2, '0') + ss_int.padStart(2, '0') + ss_decimal;
    return parseFloat(dms_str);
  }
  /**
   * @return {InterfaceProjection}
   */
  _getLatlonProjection(){
    switch(this.latLonProjectionName){
    case 'JGD2011':
      return new this.proj4.Proj('EPSG:6668');
    case 'JGD2000':
      return new this.proj4.Proj('EPSG:4612');  
    }
  }
  /**
   * @return {InterfaceProjection}
   */
  _getPlaneRectangularProjection(ref_number){
    switch (this.latLonProjectionName) {
    case 'JGD2011':
      return new this.proj4.Proj('EPSG:'+ (6668 + ref_number));
    case 'JGD2000':
      return new this.proj4.Proj('EPSG:'+ (2442 + ref_number));
    }
  }

}

const locationUtil = new LocationUtil();
export default locationUtil;