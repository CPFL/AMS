import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';
import { bindActionCreators } from 'redux';

import Radio from '@material-ui/core/Radio';
import RadioGroup from '@material-ui/core/RadioGroup';
import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import CardActions from '@material-ui/core/CardActions';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import Button from '@material-ui/core/Button';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import Typography from '@material-ui/core/Typography';

import * as ScheduleEditorActions from '../../../../../redux/Actions/ScheduleEditorActions';
import { steps } from '../../../../../model/Redux/Page/ScheduleEditor';
import ListItemText from '@material-ui/core/ListItemText/ListItemText';

class SelectStartPointComponent extends React.Component {
  constructor(props) {
    super(props);
    this.confirm = this.confirm.bind(this);
  }

  confirm() {
    if (this.props.changeRouteStartPoint !== '' || true) {
      this.props.scheduleEditorActions.setChangeRouteActiveStepNext(this.props.changeRouteActiveStep);
    } else {
      alert('Start point is not selected!');
    }
  }

  render() {
    return (
      <Card shadow={0} style={{ width: '100%', minHeight: '100px' }}>
        <CardHeader title="Select Start Point" />
        <CardContent>
          <Typography variant="subtitle1">
            <strong>
              Selected Point ID: {this.props.changeRouteStartPoint}
            </strong>
          </Typography>
        </CardContent>
        <CardActions>
          <div style={{ marginLeft: 'auto' }}>
            <Button
              color="primary"
              onClick={this.confirm}
              style={{ marginLeft: '5px' }}
            >
              Confirm
            </Button>
          </div>
        </CardActions>
      </Card>
    );
  }
}
SelectStartPointComponent.propTypes = {
  changeRouteActiveStep: PropTypes.string,
  changeRouteStartPoint: PropTypes.string,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelectStartPoint = state => ({
  changeRouteActiveStep: state.scheduleEditor.getChangeRouteActiveStep(),
  changeRouteStartPoint: state.scheduleEditor.getChangeRouteStartPoint()
});
const mapDispatchSelectStartPoint = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export const SelectStartPoint = connect(
  mapStateSelectStartPoint,
  mapDispatchSelectStartPoint
)(SelectStartPointComponent);

class SelectLaneComponent extends React.Component {
  constructor(props) {
    super(props);
    this.back = this.back.bind(this);
    this.confirm = this.confirm.bind(this);
  }

  confirm() {
    this.props.scheduleEditorActions.setActiveStep(steps.selectLane.nextStep);
  }

  back() {
    this.props.scheduleEditorActions.backStep(steps.selectLane.previousStep);
  }

  getLaneList(laneList) {
    let resList = [];
    let key = 0;

    for (let lane of laneList) {
      if (laneList.hasOwnProperty(key)) {
        resList.push(
          <ListItem key={key}>
            <ListItemText
              primary={
                <Typography variant="subtitle1">
                  <strong>Lane ID: {lane}</strong>
                </Typography>
              }
            />
          </ListItem>
        );
        key += 1;
      }
    }
    return resList;
  }

  render() {
    return (
      <Card shadow={0} style={{ width: '100%', height: '100%' }}>
        <CardHeader title="Select Lane" id="SelectLaneStepCardHeader" />
        <CardContent
          style={{ overflowY: 'auto', height: '60%' }}
          id="SelectLaneStepCardContent"
        >
          <List>{this.getLaneList(this.props.laneList)}</List>
        </CardContent>
        <CardActions>
          <div style={{ marginLeft: 'auto' }}>
            <Button onClick={this.back}>Back</Button>
            <Button
              color="primary"
              onClick={this.confirm.bind(this)}
              style={{ marginLeft: '5px' }}
            >
              Confirm
            </Button>
          </div>
        </CardActions>
      </Card>
    );
  }
}
SelectLaneComponent.propTypes = {
  activeStep: PropTypes.string,
  laneList: PropTypes.array,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelectLane = state => ({
  activeStep: state.scheduleEditor.getActiveStep(),
  laneList: state.scheduleEditor.getLaneList()
});
const mapDispatchSelectLane = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export const SelectLane = connect(
  mapStateSelectLane,
  mapDispatchSelectLane
)(SelectLaneComponent);

class SelectEndPointComponent extends React.Component {
  constructor(props) {
    super(props);
    this.back = this.back.bind(this);
    this.confirm = this.confirm.bind(this);
  }

  confirm() {
    if (this.props.endPoint !== '') {
      this.props.scheduleEditorActions.setActiveStep(
        steps.selectEndPoint.nextStep
      );
    } else {
      alert('End point is not selected!');
    }
  }

  back() {
    this.props.scheduleEditorActions.backStep(
      steps.selectEndPoint.previousStep
    );
  }

  render() {
    return (
      <Card shadow={0} style={{ width: '100%', minHeight: '100px' }}>
        <CardHeader title="Select End Point" />
        <CardContent>
          <Typography variant="subtitle1">
            <strong>Selected Point ID: {this.props.endPoint}</strong>
          </Typography>
        </CardContent>
        <CardActions>
          <div style={{ marginLeft: 'auto' }}>
            <Button onClick={this.back}>Back</Button>
            <Button
              color="primary"
              onClick={this.confirm}
              style={{ marginLeft: '5px' }}
            >
              Confirm
            </Button>
          </div>
        </CardActions>
      </Card>
    );
  }
}
SelectEndPointComponent.propTypes = {
  activeStep: PropTypes.string,
  endPoint: PropTypes.string,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelectEndPoint = state => ({
  activeStep: state.scheduleEditor.getActiveStep(),
  endPoint: state.scheduleEditor.getEndPoint()
});
const mapDispatchSelectEndPoint = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export const SelectEndPoint = connect(
  mapStateSelectEndPoint,
  mapDispatchSelectEndPoint
)(SelectEndPointComponent);

class SelectDecisionSectionEndPointComponent extends React.Component {
  constructor(props) {
    super(props);
    this.back = this.back.bind(this);
    this.confirm = this.confirm.bind(this);
  }

  confirm() {
    if (this.props.endPoint !== '') {
      this.props.scheduleEditorActions.setActiveStep(
        steps.selectEndPoint.nextStep
      );
    } else {
      alert('End point is not selected!');
    }
  }

  back() {
    this.props.scheduleEditorActions.backStep(
      steps.selectEndPoint.previousStep
    );
  }

  render() {
    return (
      <Card shadow={0} style={{ width: '100%', minHeight: '100px' }}>
        <CardHeader title="Select End Point" />
        <CardContent>
          <Typography variant="subtitle1">
            <strong>Selected Point ID: {this.props.endPoint}</strong>
          </Typography>
        </CardContent>
        <CardActions>
          <div style={{ marginLeft: 'auto' }}>
            <Button onClick={this.back}>Back</Button>
            <Button
              color="primary"
              onClick={this.confirm}
              style={{ marginLeft: '5px' }}
            >
              Confirm
            </Button>
          </div>
        </CardActions>
      </Card>
    );
  }
}
SelectDecisionSectionEndPointComponent.propTypes = {
  activeStep: PropTypes.string,
  endPoint: PropTypes.string,
  scheduleEditorActions: PropTypes.object
};
const mapStateDecitionSectionSelectEndPoint = state => ({
  activeStep: state.scheduleEditor.getActiveStep(),
  endPoint: state.scheduleEditor.getEndPoint()
});
const mapDispatchDecitionSectionSelectEndPoint = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export const SelectDecisionSectionEndPoint = connect(
  mapStateDecitionSectionSelectEndPoint,
  mapDispatchDecitionSectionSelectEndPoint
)(SelectDecisionSectionEndPointComponent);

class ResultComponent extends React.Component {
  constructor(props) {
    super(props);
    this.save = this.save.bind(this);
    this.back = this.back.bind(this);
    this.reselect = this.reselect.bind(this);
  }

  save(routeCode) {
    this.props.scheduleEditorActions.saveRouteCode(routeCode);
  }

  saveAndAnotherSelect(routeCode) {
    this.props.scheduleEditorActions.saveAndAnotherSelectRouteCode(routeCode);
  }

  reselect() {
    this.props.scheduleEditorActions.resetRouteCode();
  }

  back() {
    this.props.scheduleEditorActions.backStep(steps.result.previousStep);
  }

  getResult(startPoint, laneList, endPoint) {
    let lastPoint = '';
    let laneString = '';
    for (const lane of laneList) {
      const points = lane.split('_');

      const [arrow, startIndex, endIndex] = this.props.isBack
        ? ['<', 1, 0]
        : ['>', 0, 1];

      laneString += points[startIndex] + arrow;
      lastPoint = points[endIndex];
    }
    laneString += lastPoint;

    return startPoint + ':' + laneString + ':' + endPoint;
  }

  validateResult() {
    const waypoints = this.props.mapData.waypoint.waypoints;
    const lane = this.props.mapData.lane;
    const startPoint = this.props.startPoint;
    const laneList = this.props.laneList;
    const endPoint = this.props.endPoint;
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
          if (!this.props.isBack) {
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

      if (!this.props.isBack && startIndex > endIndex) {
        isValidate = false;
        errorMessages.push('Start Point is behind End Point');
      } else if (this.props.isBack && startIndex < endIndex) {
        isValidate = false;
        errorMessages.push('Start Point is behind End Point');
      }
    }

    if (isValidate) {
      let res = [];
      res.push(<p style={{ color: 'blue' }}>Validation is OK!</p>);
      return res;
    } else {
      let errorList = [];
      errorList.push(<p style={{ color: 'red' }}>Validation is Fail!</p>);
      for (const errorMessage of errorMessages) {
        errorList.push(<p style={{ color: 'red' }}>{errorMessage}</p>);
      }
      return errorList;
    }
  }

  render() {
    const { startPoint, laneList, endPoint } = this.props;
    const routeCode = this.getResult(startPoint, laneList, endPoint);
    return (
      <Card shadow={0} style={{ width: '100%', minHeight: '100px' }}>
        <CardHeader title="Result" />
        <CardContent>
          <Typography variant="subtitle1">
            <div style={{ wordBreak: 'break-all' }}>
              <strong>Result: {routeCode}</strong>
            </div>
          </Typography>
          <Typography variant="subtitle1">{this.validateResult()}</Typography>
        </CardContent>
        <CardActions>
          <div style={{ marginLeft: 'auto' }}>
            <Button variant="outlined" onClick={this.back}>
              Back
            </Button>
            <Button
              variant="outlined"
              color="primary"
              onClick={() => {
                this.save(routeCode);
              }}
              style={{ marginLeft: '5px' }}
            >
              Save
            </Button>
            <Button
              variant="outlined"
              color="primary"
              onClick={() => {
                this.saveAndAnotherSelect(routeCode);
              }}
              style={{ marginLeft: '5px' }}
            >
              Save And Select Another
            </Button>
            <Button
              variant="outlined"
              color="secondary"
              onClick={this.reselect}
              style={{ marginLeft: '5px' }}
            >
              Reselect
            </Button>
          </div>
        </CardActions>
      </Card>
    );
  }
}
ResultComponent.propTypes = {
  activeStep: PropTypes.string,
  isBack: PropTypes.bool,
  startPoint: PropTypes.string,
  laneList: PropTypes.array,
  endPoint: PropTypes.string,
  mapData: PropTypes.object,
  scheduleEditorActions: PropTypes.object
};
const mapStateResult = state => ({
  activeStep: state.scheduleEditor.getActiveStep(),
  isBack: state.scheduleEditor.getIsBack(),
  startPoint: state.scheduleEditor.getStartPoint(),
  laneList: state.scheduleEditor.getLaneList(),
  endPoint: state.scheduleEditor.getEndPoint(),
  mapData: state.scheduleEditor.getMapData()
});
const mapDispatchResult = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export const Result = connect(
  mapStateResult,
  mapDispatchResult
)(ResultComponent);
