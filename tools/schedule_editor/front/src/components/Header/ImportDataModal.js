import React from 'react';
import {connect} from "react-redux";
import {bindActionCreators} from "redux";


import * as ScheduleEditorActions from "../../redux/Actions/ScheduleEditorActions";
import Dialog from '@material-ui/core/Dialog';
import DialogActions from '@material-ui/core/DialogActions';
import DialogContent from '@material-ui/core/DialogContent';
import DialogTitle from '@material-ui/core/DialogTitle';
import Button from '@material-ui/core/Button';
import DoneIcon from '@material-ui/icons/Done';

import PCDLoader from '../../io/PCDLoader';


class ImportDataModal extends React.Component {

  constructor(props) {
    super(props);

    this.state = {
      waypointLoaded: false,
      laneLoaded: false,
      pcdLoaded: false,
      pcdOnLoad: false,
    };

    this.waypoint = this.lane = this.pcd = null;

    this.pcdLoader = new PCDLoader();

    this.importWaypoint = this.importWaypoint.bind(this);
    this.importLane = this.importLane.bind(this);
    this.importPCD = this.importPCD.bind(this);
    this.importMapData = this.importMapData.bind(this);
    this.handleClose = this.handleClose.bind(this);
  }

  importWaypoint(e){
    const fileList = e.target.files;
    this.waypoint = {};

    if (fileList.length) {
      const file = fileList[0];
      if (!file.name.match(/.+.json$/)) {
        e.target.value = "";
        alert('You cannot set except json file.');
      } else {
        const fileReader = new FileReader();
        fileReader.onload = event => {
          this.waypoint = JSON.parse(event.target.result);
          this.setState({waypointLoaded: true});
          console.log(this.waypoint);
        };
        fileReader.readAsText(file);
      }
    }else{
      this.setState({waypointLoaded: false});
    }
  }

  importLane(e){
    const fileList = e.target.files;
    this.lane = {};

    if (fileList.length) {
      const file = fileList[0];
      if (!file.name.match(/.+.json$/)) {
        e.target.value = "";
        alert('You cannot set except json file.');
      } else {
        const fileReader = new FileReader();
        fileReader.onload = event => {
          this.lane = JSON.parse(event.target.result);
          this.setState({laneLoaded: true});

          console.log(this.lane);
        };
        fileReader.readAsText(file);
      }
    }else{
      this.setState({laneLoaded: false});
    }
  }

  importPCD(e) {

    console.log(e);
    this.pcd = {};
    if (e.target.files.length > 0) {
      const fileList = e.target.files;
      let isPCDFileCheck = true;
      for (const file of fileList) {
        isPCDFileCheck = file.name.match(/.+.pcd$/)
      }

      if (isPCDFileCheck) {
        this.setState({pcdOnLoad: true});
        this.pcdLoader.loadFromLocalFile(fileList).then(pcd => {
          this.pcd = pcd;
          this.setState({pcdOnLoad: false, pcdLoaded: true});
          console.log(fileList, e);
        })
      } else {
        e.target.value = "";
        alert('You cannot set except pcd file.');
      }
    }else{
      this.setState({pcdLoaded: false});
    }
  }

  importMapData(){
    this.props.scheduleEditorActions.setMapData(this.pcd, this.waypoint, this.lane);
    this.props.scheduleEditorActions.setIsImportDataModalOpen(false);
  }

  handleClose() {
    this.props.scheduleEditorActions.setIsImportDataModalOpen(false);
  };

  render() {
    const {
      waypointLoaded,
      laneLoaded,
      pcdLoaded,
      pcdOnLoad
    } = this.state;
    const isLoaded = waypointLoaded && laneLoaded && !pcdOnLoad;

    return (
      <Dialog
        open={this.props.isImportDataModalOpen}
        onClose={this.handleClose}
        aria-labelledby="form-dialog-title"
        fullWidth={true}
        maxWidth='sm'
      >
        <DialogTitle id="form-dialog-title">Select Map Data</DialogTitle>
        <DialogContent>
          <input
            id="importWaypoint"
            multiple
            type="file"
            style={{display: 'none'}}
            onChange={this.importWaypoint}
          />
          <label htmlFor="importWaypoint">
            <Button
              variant="contained"
              color="primary"
              component="span"
              style={{width: '100%'}}
            >
              {waypointLoaded ? (<DoneIcon/>):""}
              Select Waypoint(Required)
            </Button>
          </label>
          <br/>
          <input
            id="importLane"
            multiple
            type="file"
            style={{display: 'none'}}
            onChange={this.importLane}
          />
          <label htmlFor="importLane">
            <Button
              variant="contained"
              color="primary"
              component="span"
              style={{marginTop: '5px', width: '100%'}}
            >
              {laneLoaded ? (<DoneIcon/>):""}
              Select Lane(Required)
            </Button>
          </label>
          <br/>
          <input
            id="importPCD"
            multiple
            type="file"
            style={{display: 'none'}}
            onChange={this.importPCD}
          />
          <label htmlFor="importPCD">
            <Button
              variant="contained"
              color="primary"
              component="span"
              style={{marginTop: '5px', width: '100%'}}
            >
              {pcdLoaded ? (<DoneIcon/>):""}
              Select PCD
            </Button>
          </label>
        </DialogContent>
        <DialogActions>
          <Button onClick={this.handleClose} color="primary">
            Cancel
          </Button>
          <Button onClick={this.importMapData} color="primary" disabled={!isLoaded}>
            Import
          </Button>
        </DialogActions>
      </Dialog>
    );
  }
}

const mapState = (state) => ({
  isImportDataModalOpen: state.scheduleEditor.getIsImportDataModalOpen()
});

const mapDispatch = (dispatch) => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch),

});

export default connect(mapState, mapDispatch)(ImportDataModal);

