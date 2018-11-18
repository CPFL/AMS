import React from 'react';
import {Button} from 'react-mdl';
import {connect} from "react-redux";

import * as RouteCodeEditorActions from "../../../redux/Actions/RouteCodeEditorActions";
import {bindActionCreators} from "redux";

import PCDLoader from '../../../io/PCDLoader';

class ImportFile extends React.Component {

  constructor(props) {
    super(props);

    this.waypoint = {};
    this.lane = {};
    this.pcd = {};

    this.pcdLoader = new PCDLoader();

  }

  loadWaypoint(e){
    let fileList = e.target.files;
    this.waypoint = {};

    if (fileList.length > 0) {
      let file = fileList[0];
      if (!file.name.match(/.+.json$/)) {
        e.target.value = "";
        alert('You cannot set except json file.');
      } else {
        let fileReader = new FileReader();
        fileReader.onload = event => {
          this.waypoint = JSON.parse(event.target.result);
        };
        fileReader.readAsText(file);
      }
    }
  }

  loadLane(e){
    let fileList = e.target.files;
    this.lane = {};

    if (fileList.length > 0) {
      let file = fileList[0];
      if (!file.name.match(/.+.json$/)) {
        e.target.value = "";
        alert('You cannot set except json file.');
      } else {
        let fileReader = new FileReader();
        fileReader.onload = event => {
          this.lane = JSON.parse(event.target.result);
        };
        fileReader.readAsText(file);
      }
    }
  }

  loadPCD(e) {

    this.pcd = {};
    if (e.target.files.length > 0) {
      let fileList = e.target.files;
      let isPCDFileCheck = true;
      for (const file of fileList) {
        isPCDFileCheck = file.name.match(/.+.pcd$/)
      }

      if (isPCDFileCheck) {
        this.pcdLoader.loadFromLocalFile(fileList).then(pcd => {
          this.pcd = pcd;
          console.log(fileList, e);
        })
      }else{
        e.target.value = "";
        alert('You cannot set except pcd file.');
      }
    }
  }

  reflectMapData (){
    this.props.routeCodeEditorActions.setMapData(this.pcd, this.waypoint, this.lane);
  }

  clear (){
    this.props.routeCodeEditorActions.setMapData({}, {}, {});
  }

  render() {

    let label_input = {
      display: "none"
    };

    return (
      <div>
        <label htmlFor="pcd_reader" className="file_reader">
          Select PCD
          <input type="file" style={label_input} id="pcd_reader" multiple onChange={this.loadPCD.bind(this)}/>
        </label>
        <label htmlFor="waypoints_reader" className="file_reader">
          Select Waypoint
          <input type="file" style={label_input} id="waypoints_reader" onChange={this.loadWaypoint.bind(this)}/>
        </label>
        <label htmlFor="lane_reader" className="file_reader">
          Select Lane
          <input type="file" style={label_input} id="lane_reader" onChange={this.loadLane.bind(this)}/>
        </label>
        <Button style={{marginLeft: "20px"}} onClick={this.reflectMapData.bind(this)}>Import Select Data</Button>
        <Button onClick={this.clear.bind(this)}>Clear</Button>
      </div>
    );
  }
}

const mapState = () => ({});

const mapDispatch = (dispatch) => ({
  routeCodeEditorActions: bindActionCreators(RouteCodeEditorActions, dispatch),

});

export default connect(mapState, mapDispatch)(ImportFile);