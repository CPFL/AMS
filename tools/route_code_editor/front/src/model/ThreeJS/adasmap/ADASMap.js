import {Road} from './road/index';
import {RoadShape} from './road_shape/index';
import {RoadSurface} from './road_surface/index';
import {RoadStructure} from './road_structure/index';
import {Roadside} from './roadside/index';
import {Point, Line, Vector, Pole, Area} from './geometry/index';
import ADASMapIndex from '../../../io/ADASMap/ADASMapIndex';

/**
 * ADASMapLoader
 */
export default class ADASMap extends THREE.Group {

    constructor(name) {
        super();
        this.name = name;
        // データの種類ごとに分割して管理
        this.road = null;
        this.road_surface = null;
        this.road_shape = null;
        this.road_structure = null;
        this.roadside = null;
        this.max_id = {
            point: 0,
            line: 0,
            vector: 0,
            pole: 0,
            area: 0,
        };
        this.ref = -1;
        this.index = null; //読み込みデータとファイル名の関連付けなどを行うインスタンス。書き出し時に使用
    }

    deleteADASMap() {
        for (let i = this.children.length - 1; i >= 0; i--) {
            this.remove(this.children[i]);
        }
    }

    addADASMap(adasmap) {
        this.deleteADASMap();
        this.add(adasmap);
    }

    async loadADASMap(adasmap) {

        if (adasmap !== null) {
            this.addADASMap(adasmap);
        } else {
            this.deleteADASMap();
        }
    }


    /**
     * @param {object} data
     * {
     *   point_data_list: array of PointData,
     *   node_data_list: array of NodeData,
     *   lane_data_list: array of LaneData,
     *   ...
     * }
     * @param {string} name
     * @param {ADASMapIndex} adasmap_index
     */
    static load(data, name, adasmap_index) {
        let adasmap = new ADASMap(name);
        adasmap.index = adasmap_index;
        let models = {
            points: [],
            lines: [],
            vectors: [],
            poles: [],
            areas: [],
        };
        // 各データから共通して参照される基本図形クラスのオブジェクト(Point, Line, Vector, Pole, Area)のインスタンスを生成
        // Generate instances of Basic geometry data because these data refered from several road data.

        // Generate Point instances
        if (data.point_data_list.length > 0) {
            let points = data.point_data_list.map(point_data => new Point(point_data));
            models.points = points;
            adasmap.max_id.point = Math.max.apply(null, data.point_data_list.map((point_data) => parseInt(point_data.pid, 10)));
            adasmap.ref = data.point_data_list[0].ref;
        }
        // Generate Line instances
        if (models.points.length > 0 && data.line_data_list.length > 0) {
            let lines = data.line_data_list.map((line_data) => {
                let before_point = models.points.find(point => point.data.pid === line_data.bpid);
                let foward_point = models.points.find(point => point.data.pid === line_data.fpid);
                if (before_point == null || foward_point == null) {
                    console.error('Line initialization error');
                    console.error('line_data', line_data);
                    console.error('before', before_point);
                    console.error('foward', foward_point);
                } else {
                    return new Line(line_data, before_point, foward_point);
                }
            });
            lines.forEach((line) => {
                let before_line = lines.find(l => l.data.lid === line.data.blid);
                let foward_line = lines.find(l => l.data.lid === line.data.flid);
                line.initializeRelatedLines(before_line, foward_line);
            });
            models.lines = lines;
            adasmap.max_id.line = Math.max.apply(null, data.line_data_list.map((line_data) => parseInt(line_data.lid, 10)));
        }
        // Generate Vector instances
        if (models.points.length > 0 && data.vector_data_list.length > 0) {
            let vectors = data.vector_data_list.map((vector_data) => {
                let point = models.points.find(point => point.data.pid === vector_data.pid);
                if (point == null) {
                    console.error('Vector initialization error');
                    console.error('vector_data', vector_data);
                    console.error('point', point);// null
                } else {
                    return new Vector(vector_data, point);
                }
            });
            models.vectors = vectors;
            adasmap.max_id.vector = Math.max.apply(null, data.vector_data_list.map(vector_data => parseInt(vector_data.vid, 10)));
        }
        // Generate Pole instance
        if (models.vectors.length > 0 && data.pole_data_list.length > 0) {
            let poles = data.pole_data_list.map((pole_data) => {
                let vector = models.vectors.find(vector => vector.data.vid === pole_data.vid);
                if (vector == null) {
                    console.error('Pole initialization error');
                    console.error('pole_data', pole_data);
                    console.error('vector', vector);// null
                } else {
                    return new Pole(pole_data, vector);
                }
            });
            models.poles = poles;
            adasmap.max_id.pole = Math.max.apply(null, data.pole_data_list.map(pole_data => parseInt(pole_data.plid, 10)));
        }
        // Generate Area instance
        if (models.lines.length > 0 && data.area_data_list.length > 0) {
            let areas = data.area_data_list.map((area_data) => {
                let lines = [];
                // start_lineからend_lineまでのlineを配列linesに格納
                let current_line = models.lines.find(line => line.data.lid === area_data.slid);
                lines.push(current_line);
                let end_line = models.lines.find(line => line.data.lid === area_data.elid);
                while (1) {
                    current_line = models.lines.find(line => line.data.lid === current_line.data.flid);
                    lines.push(current_line);
                    if (current_line === end_line) {
                        break;
                    }
                }
                return new Area(area_data, lines);
            });
            models.areas = areas;
            adasmap.max_id.area = Math.max.apply(null, data.area_data_list.map(area_data => parseInt(area_data.aid, 10)));
        }
        // Generate Road, Road shape, Road surface, Road Structure
        adasmap.road = Road.load(models, data);
        adasmap.road_shape = RoadShape.load(models, data);
        adasmap.road_surface = RoadSurface.load(models, data);
        adasmap.road_structure = RoadStructure.load(models, data);
        adasmap.roadside = Roadside.load(models, data);
        adasmap.add(adasmap.road);
        adasmap.add(adasmap.road_shape);
        adasmap.add(adasmap.road_surface);
        adasmap.add(adasmap.road_structure);
        adasmap.add(adasmap.roadside);
        return adasmap;
    }

    static create(name) {
        let adasmap = new ADASMap(name);
        adasmap.index = new ADASMapIndex();
        adasmap.road = new Road();
        adasmap.road_shape = new RoadShape();
        adasmap.road_surface = new RoadSurface();
        adasmap.road_structure = new RoadStructure();
        adasmap.roadside = new Roadside();
        adasmap.add(adasmap.road);
        adasmap.add(adasmap.road_shape);
        adasmap.add(adasmap.road_surface);
        adasmap.add(adasmap.road_structure);
        adasmap.add(adasmap.roadside);
        return adasmap;
    }

    // max_id.name を1増加し、その値を返す
    incrementMaxId(name) {
        if (this.max_id[name] === undefined) {
            this.max_id[name] = 0;
        }
        this.max_id[name] += 1;
        return this.max_id[name];
    }

}
