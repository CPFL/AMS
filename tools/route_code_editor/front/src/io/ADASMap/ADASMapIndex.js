/**
 * Object of ADASMap index file(idx.csv)
 */
/**
 *
 */
export default class ADASMapIndex {

    constructor(idx_csv) {
        this.data_definitions = [
            // Basic geometry
            {kind: 'B01', data_name: 'point', file_name: 'point.csv', includ_idx: false},
            {kind: 'B02', data_name: 'vector', file_name: 'vector.csv', includ_idx: false},
            {kind: 'B03', data_name: 'pole', file_name: 'pole.csv', includ_idx: false},
            {kind: 'B04', data_name: 'line', file_name: 'line.csv', includ_idx: false},
            {kind: 'B05', data_name: 'area', file_name: 'area.csv', includ_idx: false},
            // 未使用,予約クラス{ kind: 'B06', data_name: 'box', file_name: 'box.csv' },
            // Road
            {kind: 'N01', data_name: 'dtlane', file_name: 'dtlane.csv', includ_idx: true},
            {kind: 'N02', data_name: 'node', file_name: 'node.csv', includ_idx: true},
            {kind: 'N03', data_name: 'lane', file_name: 'lane.csv', includ_idx: true},
            {kind: 'N04', data_name: 'wayarea', file_name: 'wayarea.csv', includ_idx: true},
            // Road shape
            {kind: 'R001', data_name: 'roadedge', file_name: '', includ_idx: true},
            {kind: 'R002', data_name: 'gutter', file_name: '', includ_idx: true},
            {kind: 'R003', data_name: 'curb', file_name: '', includ_idx: true},
            {kind: 'R004', data_name: 'intersection', file_name: '', includ_idx: true},
            // 未定義: 'R005', data_name: 'concrete', 路肩コンクリート
            // Road surface
            {kind: 'P001', data_name: 'whiteline', file_name: '', includ_idx: true},
            {kind: 'P002', data_name: 'stopline', file_name: '', includ_idx: true},
            {kind: 'P003', data_name: 'zebrazone', file_name: '', includ_idx: true},
            {kind: 'P004', data_name: 'crosswalk', file_name: '', includ_idx: true},
            {kind: 'P005', data_name: 'road_surface_mark', file_name: '', includ_idx: true},
            // Road side
            {kind: 'S001', data_name: 'guardrail', file_name: '', includ_idx: true},
            {kind: 'S002', data_name: 'sidewalk', file_name: '', includ_idx: true},
            // 未定義: 'S003': 車両乗り入れ部
            // Structure
            {kind: 'K001', data_name: 'poledata', file_name: '', includ_idx: true},
            {kind: 'K002', data_name: 'utilitypole', file_name: '', includ_idx: true},
            {kind: 'K003', data_name: 'roadsign', file_name: '', includ_idx: true},
            {kind: 'K004', data_name: 'signaldata', file_name: '', includ_idx: true},
            {kind: 'K005', data_name: 'streetlight', file_name: '', includ_idx: true},
            {kind: 'K006', data_name: 'curvemirror', file_name: '', includ_idx: true},
            {kind: 'K007', data_name: 'wall', file_name: '', includ_idx: true},
            {kind: 'K008', data_name: 'fence', file_name: '', includ_idx: true},
            {kind: 'K009', data_name: 'railroad_crossing', file_name: '', includ_idx: true},
        ];

        /*
          this.data_definitions = [
              // Basic geometry
              { kind: 'B01', data_name: 'point', file_name: 'point.csv', includ_idx: false },
              { kind: 'B02', data_name: 'vector', file_name: 'vector.csv', includ_idx: false },
              { kind: 'B03', data_name: 'pole', file_name: 'pole.csv', includ_idx: false },
              { kind: 'B04', data_name: 'line', file_name: 'line.csv', includ_idx: false },
              { kind: 'B05', data_name: 'area', file_name: 'area.csv', includ_idx: false },
              // 未使用,予約クラス{ kind: 'B06', data_name: 'box', file_name: 'box.csv' },
              // Road
              { kind: 'N01', data_name: 'dtlane', file_name: 'dtlane.csv', includ_idx: true },
              { kind: 'N02', data_name: 'node', file_name: 'node.csv', includ_idx: true },
              { kind: 'N03', data_name: 'lane', file_name: 'lane.csv', includ_idx: true },
              { kind: 'N04', data_name: 'wayarea', file_name: 'wayarea.csv', includ_idx: true },
              // Road shape
              { kind: 'R001', data_name: 'roadedge', file_name: 'roadedge.csv', includ_idx: true },
              { kind: 'R002', data_name: 'gutter', file_name: 'gutter.csv', includ_idx: true },
              { kind: 'R003', data_name: 'curb', file_name: 'curb.csv', includ_idx: true },
              { kind: 'R004', data_name: 'intersection', file_name: 'intersection.csv', includ_idx: true },
              // 未定義: 'R005', data_name: 'concrete', 路肩コンクリート
              // Road surface
              { kind: 'P001', data_name: 'whiteline', file_name: 'whiteline.csv', includ_idx: true },
              { kind: 'P002', data_name: 'stopline', file_name: 'stopline.csv', includ_idx: true },
              { kind: 'P003', data_name: 'zebrazone', file_name: 'zebrazone.csv', includ_idx: true },
              { kind: 'P004', data_name: 'crosswalk', file_name: 'crosswalk.csv', includ_idx: true },
              { kind: 'P005', data_name: 'road_surface_mark', file_name: 'road_surface_mark.csv', includ_idx: true },
              // Road side
              { kind: 'S001', data_name: 'guardrail', file_name: 'guardrail.csv', includ_idx: true },
              { kind: 'S002', data_name: 'sidewalk', file_name: 'sidewalk.csv', includ_idx: true },
              // 未定義: 'S003': 車両乗り入れ部
              // Structure
              { kind: 'K001', data_name: 'poledata', file_name: 'poledata.csv', includ_idx: true },
              { kind: 'K002', data_name: 'utilitypole', file_name: 'utilitypole.csv', includ_idx: true },
              { kind: 'K003', data_name: 'roadsign', file_name: 'roadsign.csv', includ_idx: true },
              { kind: 'K004', data_name: 'signaldata', file_name: 'signaldata.csv', includ_idx: true },
              { kind: 'K005', data_name: 'streetlight', file_name: 'streetlight.csv', includ_idx: true },
              { kind: 'K006', data_name: 'curvemirror', file_name: 'curvemirror.csv', includ_idx: true },
              { kind: 'K007', data_name: 'wall', file_name: 'wall.csv', includ_idx: true },
              { kind: 'K008', data_name: 'fence', file_name: 'fence.csv', includ_idx: true },
              { kind: 'K009', data_name: 'railroad_crossing', file_name: 'railroad_crossing.csv', includ_idx: true },
          ];
          */
        let csv = idx_csv || [];
        csv.forEach((record) => {
            let def = this.data_definitions.find((def) => def.kind === record[1]);
            if (def) {
                def.file_name = record[2];
            }
        });
    }

    getCsvByName(data_name) {
        let def = this.data_definitions.find((def) => def.data_name === data_name);
        return def.kind + ',' + def.file_name + '\n';
    }

    getDataNameByKind(kind) {
        let def = this.data_definitions.find((def) => def.kind === kind);
        if (def) {
            return def.data_name;
        } else {
            return null;
        }
    }

    getDataNameByFileName(file_name) {
        let def = this.data_definitions.find((def) => def.file_name === file_name);
        if (def) {
            return def.data_name;
        } else {
            return null;
        }
    }

    getFileName(data_name) {
        let def = this.data_definitions.find((def) => def.data_name === data_name);
        if (def && def.file_name != '') {
            return def.file_name;
        } else {
            return null;
        }
    }

    getFileNames() {

        console.log("index name", this.data_definitions);
        let file_names = this.data_definitions.filter((def) => {
            return def.file_name != '';
        }).map((def) => {
            return def.file_name;
        });
        return file_names;
    }

}
