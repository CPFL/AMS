import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';
import { bindActionCreators } from 'redux';

import * as ScheduleEditorActions from '../../redux/Actions/ScheduleEditorActions';
import { MuiThemeProvider, createMuiTheme } from '@material-ui/core/styles';
import cyan from '@material-ui/core/colors/cyan';
import Dialog from '@material-ui/core/Dialog';
import DialogActions from '@material-ui/core/DialogActions';
import DialogContent from '@material-ui/core/DialogContent';
import DialogTitle from '@material-ui/core/DialogTitle';
import Divider from '@material-ui/core/Divider';
import Typography from '@material-ui/core/Typography';
import Button from '@material-ui/core/Button';
import DoneIcon from '@material-ui/icons/Done';

import PCDLoader from '../../io/PCDLoader';

const theme = createMuiTheme({
  palette: {
    secondary: cyan
  }
});

class ImportDataModal extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      waypointLoaded: false,
      laneLoaded: false,
      pcdLoaded: false,
      pcdOnLoad: false
    };

    this.waypoint = this.lane = this.pcd = null;

    this.pcdLoader = new PCDLoader();

    //this.importWaypoint = this.importWaypoint.bind(this);
    this.importLane = this.importLane.bind(this);
    this.importPCD = this.importPCD.bind(this);
    this.importMapData = this.importMapData.bind(this);
    this.clearData = this.clearData.bind(this);
    this.handleClose = this.handleClose.bind(this);
  }

  importWaypoint = e => {
    const fileList = e.target.files;
    this.waypoint = {};

    if (fileList.length) {
      const file = fileList[0];
      if (!file.name.match(/.+.json$/)) {
        e.target.value = '';
        alert('Only pcd file is available');
      } else {
        const fileReader = new FileReader();
        fileReader.onload = event => {
          this.waypoint = JSON.parse(event.target.result);
          this.setState({ waypointLoaded: true });
        };
        fileReader.readAsText(file);
      }
    } else {
      this.setState({ waypointLoaded: false });
    }
  };

  importLane(e) {
    const fileList = e.target.files;
    this.lane = {};

    console.log(fileList);
    if (fileList.length) {
      const file = fileList[0];
      if (!file.name.match(/.+.json$/)) {
        e.target.value = '';
        alert('Only json file is available');
      } else {
        const fileReader = new FileReader();
        fileReader.onload = event => {
          this.lane = JSON.parse(event.target.result);
          this.setState({ laneLoaded: true });
        };
        fileReader.readAsText(file);
      }
    } else {
      this.setState({ laneLoaded: false });
    }
  }

  importPCD(e) {
    this.pcd = {};
    console.log(e.target.files);
    if (e.target.files.length > 0) {
      const fileList = e.target.files;
      let checkPCDFileOnly = true;
      for (const file of fileList) {
        checkPCDFileOnly = file.name.match(/.+.pcd$/);
      }

      if (checkPCDFileOnly) {
        this.setState({ pcdOnLoad: true });
        this.pcdLoader.loadFromLocalFile(fileList).then(pcd => {
          this.pcd = pcd;
          this.setState({ pcdOnLoad: false, pcdLoaded: true });
        });
      } else {
        e.target.value = '';
        alert('Only pcd file is available');
      }
    } else {
      this.setState({ pcdLoaded: false });
    }
  }

  importMapData() {
    this.props.scheduleEditorActions.setMapData(
      this.pcd,
      this.waypoint,
      this.lane
    );
    this.props.scheduleEditorActions.setIsImportDataModalOpen(false);
  }

  clearData() {
    this.waypoint = this.lane = this.pcd = null;
    const importWaypoint = document.getElementById('importWaypoint');
    importWaypoint.value = '';
    const importLane = document.getElementById('importLane');
    importLane.value = '';
    const importPCD = document.getElementById('importPCD');
    importPCD.value = '';

    this.setState({
      waypointLoaded: false,
      laneLoaded: false,
      pcdLoaded: false
    });
  }

  handleClose() {
    this.props.scheduleEditorActions.setIsImportDataModalOpen(false);
  }

  render() {
    const { waypointLoaded, laneLoaded, pcdLoaded, pcdOnLoad } = this.state;
    const isLoaded = waypointLoaded && laneLoaded && !pcdOnLoad;

    return (
      <Dialog
        open={this.props.isImportDataModalOpen}
        onClose={this.handleClose}
        aria-labelledby="form-dialog-title"
        fullWidth={true}
        maxWidth="sm"
      >
        <DialogTitle id="form-dialog-title">Select Map Data</DialogTitle>
        <DialogContent>
          <MuiThemeProvider theme={theme}>
            <div style={{ marginBottom: '20px' }}>
              <Typography gutterBottom variant="h6" style={{ color: 'red' }}>
                Required
              </Typography>
              <input
                id="importWaypoint"
                multiple
                type="file"
                style={{ display: 'none' }}
                onChange={this.importWaypoint}
              />
              <label htmlFor="importWaypoint">
                <Button
                  variant="contained"
                  color={waypointLoaded ? 'secondary' : 'primary'}
                  component="span"
                  style={{ width: '100%' }}
                >
                  {waypointLoaded ? <DoneIcon /> : ''}
                  Select Waypoint
                </Button>
              </label>
              <br />
              <input
                id="importLane"
                multiple
                type="file"
                style={{ display: 'none' }}
                onChange={this.importLane}
              />
              <label htmlFor="importLane">
                <Button
                  variant="contained"
                  color={laneLoaded ? 'secondary' : 'primary'}
                  component="span"
                  style={{ marginTop: '5px', width: '100%' }}
                >
                  {laneLoaded ? <DoneIcon /> : ''}
                  Select Lane
                </Button>
              </label>
            </div>
            <Divider variant="middle" />
            <div>
              <Typography gutterBottom variant="h6" style={{ color: 'black' }}>
                Option
              </Typography>
              <input
                id="importPCD"
                multiple
                type="file"
                style={{ display: 'none' }}
                onChange={this.importPCD}
              />
              <label htmlFor="importPCD">
                <Button
                  variant="contained"
                  color={pcdLoaded ? 'secondary' : 'primary'}
                  component="span"
                  style={{ marginTop: '5px', width: '100%' }}
                >
                  {pcdLoaded ? <DoneIcon /> : ''}
                  Select PCD
                </Button>
              </label>
            </div>
          </MuiThemeProvider>
        </DialogContent>
        <DialogActions>
          <Button onClick={this.handleClose}>Cancel</Button>
          <Button onClick={this.clearData} disabled={pcdOnLoad}>
            Clear Data
          </Button>
          <Button
            onClick={this.importMapData}
            color="primary"
            disabled={!isLoaded}
          >
            Import
          </Button>
        </DialogActions>
      </Dialog>
    );
  }
}
ImportDataModal.propTypes = {
  isImportDataModalOpen: PropTypes.bool,
  scheduleEditorActions: PropTypes.object
};
const mapState = state => ({
  isImportDataModalOpen: state.scheduleEditor.getIsImportDataModalOpen()
});

const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapState,
  mapDispatch
)(ImportDataModal);
