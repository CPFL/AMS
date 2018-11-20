import parse from 'csv-parse';
import ADASMap from '../../model/ThreeJS/adasmap/ADASMap';
// basic geometry
import {PointData, LineData, VectorData, PoleData, AreaData} from '../../model/ThreeJS/adasmap/geometry/index';
// road
import {NodeData, LaneData, DtlaneData, WayareaData} from '../../model/ThreeJS/adasmap/road/index';
// road shape
import {CurbData, RoadedgeData, GutterData, IntersectionData} from '../../model/ThreeJS/adasmap/road_shape/index';
// road surface
import {
    WhitelineData,
    StoplineData,
    ZebrazoneData,
    CrosswalkData,
    RoadSurfaceMarkData
} from '../../model/ThreeJS/adasmap/road_surface/index';
// road side
import {GuardrailData, SidewalkData} from '../../model/ThreeJS/adasmap/roadside/index';
// road structure
import {
    PoledataData, UtilitypoleData, RoadsignData, SignaldataData,
    StreetlightData, CurvemirrorData, WallData, FenceData, RailroadCrossingData
} from '../../model/ThreeJS/adasmap/road_structure/index';
import ADASMapIndex from './ADASMapIndex';

import {ADASMAP_LOADER} from '../../constants/Constant'

/**
 * @example
 * let loader = new ADASMapLoader();
 * let file_list = loader.prepare(file_list);
 * loader.parse(name).then((adasmap)=>{
 *   scene.add(adasmap);
 * });
 * @description
 * -------------
 * Specification
 * -------------
 * Manifest(Index)
 * - idx.csv: manifest of feature data file name インデックスファイル
 * Basic Geometry 基本図形クラス
 * - B01 point.csv ポイント
 * - B02 vector.csv ベクトル
 * - B03 pole.csv ポール(円柱)
 * - B04 line.csv ライン
 * - B05 area.csv エリア
 * - B06 box.csv ボックス
 * Road 道路データ
 * - N01 dtlane.csv 中心線形
 * - N02 node.csv ノード
 * - N03 lane.csv レーン
 * - N04 wayarea.csv 走行エリア
 ** Below files file name are declared in idx.csv
 * Road shape 地物(道路形状)
 * - R001 roadedge 道路縁
 * - R002 gutter 側溝
 * - R003 curb 縁石
 * - R004 intersection 交差点
 * - R005 ? 路肩コンクリート
 * Road surface 地物(路面)
 * - P001 whiteline 白線
 * - P002 stopline 停止線
 * - P003 zebrazone ゼブラゾーン
 * - P004 crosswalk 横断歩道
 * - P005 road_surface_mark 路面マーク
 * Road side 地物(側帯)
 * - S001 guardrail ガードレール
 * - S002 sidewalk 歩道
 * - S003 ? 車両乗入部
 * Road structure 地物(構造物)
 * - K001 poledata ポール
 * - K002 utilitypole 電柱
 * - K003 roadsign 標識
 * - K004 signaldata 信号
 * - K005 streetlight 街灯
 * - K006 curvemirror カーブミラー
 * - K007 wall 壁面
 * - K008 fence フェンス
 * - K009 railroad_crossing 踏切ゾーン
 *
 * About Async/Await and Promise
 * Please see {@link https://qiita.com/niusounds/items/37c1f9b021b62194e077}
 */
export default class ADASMapLoader {

    constructor(loadingManager) {
        this.loadingManager = loadingManager;
        this.dirPath = null;
        this.file_list = null;
        this.index = null;
    }

    /**
     * @param {array} file_list
     * @param folder_name
     * @return {array} file_list - return array of File instance. file.loadable attribute is set
     */
    async loadADASMap(file_list, folder_name) {

        if (file_list !== null && folder_name !== null) {

            let idx_file = file_list.find((file) => file.name === 'idx.csv');
            if (idx_file) {
                let idx_file_data = await this._readFile(idx_file);
                let idx_csv = await this._parseCsv(idx_file_data);
                this.index = new ADASMapIndex(idx_csv);
            } else {
                this.index = new ADASMapIndex([]);
            }
            let loadable_files = this.index.getFileNames();
            this.file_list = file_list.map((file) => {
                file.loadable = !!(loadable_files.includes(file.name) || file.name === 'idx.csv');
                return file;
            });

            let raw_data_dictionary = await this._readFileList(this.file_list);
            let data = await this._parseAllData(raw_data_dictionary);
            return ADASMap.load(data, folder_name, this.index);
        }
    }

    async fetchADASMap() {

        let idx_file_data = await this._fetchFile('idx.csv');
        let idx_csv = await this._parseCsv(idx_file_data);
        if (idx_csv) {
            this.index = new ADASMapIndex(idx_csv);
        } else {
            this.index = new ADASMapIndex([]);
        }
        let loadable_files = this.index.getFileNames();

        let raw_data_dictionary = await this._fetchFileList(loadable_files);
        let data = await this._parseAllData(raw_data_dictionary);
        return ADASMap.load(data, 'fetch_files', this.index);
    }

    /**
     * @param {array} file_list
     * @return {array} file_list - return array of File instance. file.loadable attribute is set
     */
    async prepare(file_list) {
        // idx.csv のみprepare関数内で読み込み、
        // idx.csvで宣言されている読み込み可能なファイルの名前を取得
        let idx_file = file_list.find((file) => file.name === 'idx.csv');
        if (idx_file) {
            let idx_file_data = await this._readFile(idx_file);
            let idx_csv = await this._parseCsv(idx_file_data);
            this.index = new ADASMapIndex(idx_csv);
        } else {
            this.index = new ADASMapIndex([]);
        }
        let loadable_files = this.index.getFileNames();
        this.file_list = file_list.map((file) => {
            if (loadable_files.includes(file.name) || file.name === 'idx.csv') {
                file.loadable = true;
            } else {
                file.loadable = false;
            }
            return file;
        });
        return file_list;
    }

    /**
     * @param {string} name
     * @return {ADASMap} adasmap
     */
    async parse(name) {
        let raw_data_dictionary = await this._readFileList(this.file_list);
        let data = await this._parseAllData(raw_data_dictionary);
        return ADASMap.load(data, name, this.index);
    }

    //////////////
    //   Read   //
    //////////////
    /**
     * @param {array of File} file_list
     * @return raw_data_dictionary - key: filename(without extension), value: data
     * {
     *  'point': data,
     *  'node': data,
     *  'lane': data,
     *   ...
     * }
     */
    async _readFileList(file_list) {
        let raw_data_dictionary = {};
        for (let i = 0; i < file_list.length; i++) {
            let file = file_list[i];
            if (file.loadable === true) {
                let data_name = this.index.getDataNameByFileName(file.name);
                const data = await this._readFile(file);
                raw_data_dictionary[data_name] = data;
            }
        }
        return raw_data_dictionary;
    }

    /**
     * Read file as text data
     * @param {File} file
     * @return {string} data
     */
    async _readFile(file) {
        return new Promise((resolve) => {
            let fileReader = new FileReader();
            fileReader.onload = event => {
                resolve(event.target.result);
            };
            fileReader.readAsText(file);
        });
    }


    async _fetchFileList(file_list) {
        let raw_data_dictionary = {};
        for (const fileName of file_list) {
            let data_name = this.index.getDataNameByFileName(fileName);
            const data = await this._fetchFile(fileName);
            if(data !== null) {
                raw_data_dictionary[data_name] = data;
            }
        }
        return raw_data_dictionary;
    }

    async _fetchFile(fileName) {
        return new Promise((resolve) => {
            fetch(ADASMAP_LOADER.URL_PREFIX + fileName)
                .then((response) => {
                    if (response.ok) {
                        return response.text()
                    } else {
                        resolve(null)
                    }
                })
                .then((text) => resolve(text))
                .catch((error) => {
                    console.log(error);
                    resolve(null)
                });
        });
    }


    //////////////
    //  Parse   //
    //////////////
    async _parseAllData(raw_data_dictionary) {
        let data = {
            /**
             * Basic Geometry
             */
            'point_data_list': [],
            'vector_data_list': [],
            'pole_data_list': [],
            'line_data_list': [],
            'area_data_list': [],
            //'box_data_list': null,
            /**
             * Road Data
             */
            //'dtlane_data_list': null,
            'node_data_list': [],
            'lane_data_list': [],
            'wayarea_data_list': [],
            /**
             * Feature of road shape
             */
            'roadedge_data_list': [],
            'gutter_data_list': [],
            'curb_data_list': [],
            'intersection_data_list': [],
            /**
             * Feature of road surface
             */
            'whiteline_data_list': [],
            'stopline_data_list': [],
            'zebrazone_data_list': [],
            'crosswalk_data_list': [],
            'road_surface_mark_data_list': [],
            /**
             * Feature of roadside
             */
            'guardrail_data_list': [],
            'sidewalk_data_list': [],
            /**
             * Feature of structure
             */
            'poledata_data_list': [],
            'utilitypole_data_list': [],
            'roadsign_data_list': [],
            'signaldata_data_list': [],
            'streetlight_data_list': [],
            'curvemirror_data_list': [],
            'wall_data_list': [],
            'fence_data_list': [],
            'railroad_crossing_data_list': [],

        };
        const lodableFiles = [
            ['point', PointData], ['vector', VectorData], ['pole', PoleData], ['line', LineData], ['area', AreaData],
            ['node', NodeData], ['lane', LaneData], ['dtlane', DtlaneData], ['wayarea', WayareaData],
            ['curb', CurbData], ['roadedge', RoadedgeData], ['gutter', GutterData], ['intersection', IntersectionData],
            ['whiteline', WhitelineData], ['stopline', StoplineData], ['zebrazone', ZebrazoneData], ['crosswalk', CrosswalkData], ['road_surface_mark', RoadSurfaceMarkData],
            ['guardrail', GuardrailData], ['sidewalk', SidewalkData],
            ['poledata', PoledataData], ['utilitypole', UtilitypoleData], ['roadsign', RoadsignData],
            ['signaldata', SignaldataData], ['streetlight', StreetlightData], ['curvemirror', CurvemirrorData],
            ['wall', WallData], ['fence', FenceData], ['railroad_crossing', RailroadCrossingData],
        ];
        let async_functions = lodableFiles.map((lodableFile) => {
            return async () => {
                const name = lodableFile[0];
                const DataClass = lodableFile[1];
                if (raw_data_dictionary[name] != null) {
                    const csv = await this._parseCsv(raw_data_dictionary[name]);
                    data[name + '_data_list'] = await this._parseData(csv, DataClass);
                }
            };
        });
        await Promise.all(async_functions.map(p => p()));
        console.log('_parseAllData() result: data');
        return data;
    }

    /**
     * Convert text data to csv records
     * @param {string} raw data(text data)
     * @return {array} Array of csv record
     */
    async _parseCsv(raw_data) {
        return new Promise((resolve, reject) => {
            parse(raw_data, (error, output) => {
                if (error) {
                    reject(error);
                } else {
                    resolve(output);
                }
            });
        });
    }

    /**
     * Convert csv records to Instances of Data Class
     * @param {array} csv
     * @param {class} DataClass - (ex, PointData, NodeData, LaneData)
     * @return {array} data_list of DataClass instance
     */
    async _parseData(csv, DataClass) {
        csv.shift(); // Delete header(column names)
        let data_list = [];
        csv.forEach((record) => {
            data_list.push(new DataClass(...record));
        });
        return data_list;
    }

}  
