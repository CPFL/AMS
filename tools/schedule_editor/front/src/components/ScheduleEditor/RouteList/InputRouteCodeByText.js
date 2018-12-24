import React from 'react';
import PropTypes from 'prop-types';

import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';

import connect from 'react-redux/es/connect/connect';
import { bindActionCreators } from 'redux';
import * as ScheduleEditorActions from '../../../redux/Actions/ScheduleEditorActions';
import Dialog from '@material-ui/core/Dialog/Dialog';
import DialogTitle from '@material-ui/core/DialogTitle/DialogTitle';
import DialogContent from '@material-ui/core/DialogContent';
import DialogActions from '@material-ui/core/DialogActions';

const advanceCharacter = '>';
const backCharacter = '<';

class InputRouteCodeByText extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      textRouteCode: ''
    };

    this.setTextRouteCode = this.setTextRouteCode.bind(this);
    this.closeAddRouteByTextModal = this.closeAddRouteByTextModal.bind(this);
    this.setTextRouteCode = this.setTextRouteCode.bind(this);

    this.checkAndInput = this.checkAndInput.bind(this);
  }

  closeAddRouteByTextModal() {
    this.props.scheduleEditorActions.setIsAddRouteByTextModalOpen(false);
  }
  setTextRouteCode(event) {
    this.setState({ textRouteCode: event.target.value });
  }

  checkAndInput() {
    const textRouteCode = this.state.textRouteCode;
    let isBack = false,
      directionCharacter = advanceCharacter;
    if (
      textRouteCode.match(/[1-9][0-9]*:[1-9][0-9]*(>[1-9][0-9]*)+:[1-9][0-9]*/)
    ) {
      [isBack, directionCharacter] = [false, advanceCharacter];
    } else if (
      textRouteCode.match(/[1-9][0-9]*:[1-9][0-9]*(<[1-9][0-9]*)+:[1-9][0-9]*/)
    ) {
      [isBack, directionCharacter] = [false, backCharacter];
    } else {
      alert('Format Error');
      return;
    }
    const [startPoint, laneString, endPoint] = textRouteCode.split(':');
    const lanePointList = laneString.split(directionCharacter);
    const laneList = [];

    lanePointList.forEach((point, index) => {
      if (lanePointList[index + 1]) {
        laneList.push(point + '_' + lanePointList[index + 1]);
      }
    });

    console.log(startPoint, laneList, endPoint, isBack);
    console.log(textRouteCode);

    if (this.validateResult(startPoint, laneList, endPoint, isBack)) {
      this.props.scheduleEditorActions.addRouteCodeByText({
        startPoint: startPoint,
        laneList: laneList,
        endPoint: endPoint,
        isBack: isBack,
        routeCode: textRouteCode
      });
    } else {
      alert('Validation is false');
    }
  }

  validateResult(startPoint, laneList, endPoint, isBack) {
    const waypoints = this.props.mapData.waypoint.waypoints;
    const lane = this.props.mapData.lane;
    let errorMessages = [];
    let isValidate = true;

    if (!waypoints.hasOwnProperty(startPoint)) {
      isValidate = false;
      errorMessages.push('Waypoints does not have this Start Point');
    }
    for (const laneID of laneList) {
      if (!lane.lanes.hasOwnProperty(laneID)) {
        isValidate = false;
        errorMessages.push('Lanes does not have this LaneID:' + laneID);
      } else {
        const index = laneList.indexOf(laneID);
        if (index === 0) {
          if (lane.lanes[laneID].waypointIDs.indexOf(startPoint) === -1) {
            isValidate = false;
            errorMessages.push('Start Point does not exist on the first lane');
          }
        } else {
          if (!isBack) {
            if (
              lane.toLanes[laneList[index - 1]].indexOf(laneList[index]) === -1
            ) {
              isValidate = false;
              errorMessages.push(
                'This Lane does not exist in toLanes of previous lane: LaneID is ' +
                  laneID
              );
            }
          } else {
            if (
              lane.fromLanes[laneList[index - 1]].indexOf(laneList[index]) ===
              -1
            ) {
              isValidate = false;
              errorMessages.push(
                'This Lane does not exist in fromLanes of previous lane: LaneID is ' +
                  laneID
              );
            }
          }
        }
      }
    }

    if (lane.lanes.hasOwnProperty(laneList[laneList.length - 1])) {
      if (
        lane.lanes[laneList[laneList.length - 1]].waypointIDs.indexOf(
          endPoint
        ) === -1
      ) {
        isValidate = false;
        errorMessages.push('End Point does not exist on the last lane');
      }
    }

    if (laneList.length === 1 && lane.lanes.hasOwnProperty(laneList[0])) {
      const startIndex = lane.lanes[laneList[0]].waypointIDs.indexOf(
        startPoint
      );
      const endIndex = lane.lanes[laneList[0]].waypointIDs.indexOf(endPoint);

      if (!isBack && startIndex > endIndex) {
        isValidate = false;
        errorMessages.push('Start Point is behind End Point');
      } else if (isBack && startIndex < endIndex) {
        isValidate = false;
        errorMessages.push('Start Point is behind End Point');
      }
    }

    console.log(errorMessages);
    return isValidate;
  }

  render() {
    return (
      <Dialog
        open={this.props.isAddRouteByTextModalOpen}
        onClose={this.closeAddRouteByTextModal}
      >
        <DialogTitle id="alert-dialog-slide-title">
          Input Route Code
        </DialogTitle>
        <DialogContent>
          <div style={{ marginLeft: 'auto' }}>
            <TextField
              id="standard-dense"
              label="Input Route Code"
              margin="dense"
              onChange={this.setTextRouteCode}
              style={{ width: '200px' }}
            />
          </div>
        </DialogContent>
        <DialogActions>
          <Button onClick={this.closeAddRouteByTextModal} color="primary">
            Cancel
          </Button>
          <Button
            color="primary"
            variant="contained"
            onClick={this.checkAndInput}
          >
            Confirm
          </Button>
        </DialogActions>
      </Dialog>
    );
  }
}

InputRouteCodeByText.propTypes = {
  mapData: PropTypes.object,
  isAddRouteByTextModalOpen: PropTypes.bool,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelect = state => ({
  mapData: state.scheduleEditor.getMapData(),
  isAddRouteByTextModalOpen: state.scheduleEditor.getIsAddRouteByTextModalOpen()
});
const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapStateSelect,
  mapDispatch
)(InputRouteCodeByText);
