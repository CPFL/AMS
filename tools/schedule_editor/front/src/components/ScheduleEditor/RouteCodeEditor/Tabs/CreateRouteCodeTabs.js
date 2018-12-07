import React from 'react';

import Radio from '@material-ui/core/Radio';
import RadioGroup from '@material-ui/core/RadioGroup';
import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardMedia from '@material-ui/core/CardMedia';
import CardContent from '@material-ui/core/CardContent';
import CardActions from '@material-ui/core/CardActions';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import Button from '@material-ui/core/Button';

import {connect} from "react-redux";
import * as ScheduleEditorActions from "../../../../redux/Actions/ScheduleEditorActions";
import {bindActionCreators} from "redux";

import {steps} from '../../../../model/Redux/Page/ScheduleEditor'

class AdvanceOrBackComponent extends React.Component {

  confirm() {
    this.props.scheduleEditorActions.setActiveStep(steps.advanceOrBack.nextStep)
  }

  setIsBack(event) {
    console.log(event);
    this.props.scheduleEditorActions.setIsBack(event.target.value === "back");
  };

  render() {
    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px"}}>
        <CardHeader title="Select Advance or Back"/>
        <CardContent>
          <RadioGroup
            name="AdvanceOrBack"
            value={this.props.isBack ? "back" : "advance"}
            onChange={this.setIsBack.bind(this)}
          >
            <FormControlLabel value="advance" control={<Radio/>} label="Advance"/>
            <FormControlLabel value="back" control={<Radio/>} label="Back"/>
          </RadioGroup>
        </CardContent>
        <CardActions>
          <div style={{marginLeft: "auto"}}>
            <Button onClick={this.confirm.bind(this)}>Confirm</Button>
          </div>
        </CardActions>
      </Card>
    )
  }
}

const mapStateAdvanceOrBack = (state) => ({
  activeStep: state.scheduleEditor.getActiveStep(),
  isBack: state.scheduleEditor.getIsBack()
});
const mapDispatchAdvanceOrBack = (dispatch) => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch),
});
let AdvanceOrBack = connect(mapStateAdvanceOrBack, mapDispatchAdvanceOrBack)(AdvanceOrBackComponent);


class SelectStartPointComponent extends React.Component {

  confirm() {
    if (this.props.startPoint !== "") {
      this.props.scheduleEditorActions.setActiveStep(steps.selectStartPoint.nextStep)
    } else {
      alert("Start point is not selected!");
    }
  }

  back() {
    this.props.scheduleEditorActions.backStep(steps.selectStartPoint.previousStep)
  }

  render() {
    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px"}}>
        <CardHeader title="Select Start Point"/>
        <CardContent><strong>Selected Point ID: {this.props.startPoint}</strong></CardContent>
        <CardActions>
          <div style={{marginLeft: "auto"}}>
            <Button onClick={this.back.bind(this)}>Back</Button>
            <Button onClick={this.confirm.bind(this)}>Confirm</Button>
          </div>
        </CardActions>
      </Card>

    )
  }
}

const mapStateSelectStartPoint = (state) => ({
  activeStep: state.scheduleEditor.getActiveStep(),
  startPoint: state.scheduleEditor.getStartPoint()
});
const mapDispatchSelectStartPoint = (dispatch) => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch),
});
let SelectStartPoint = connect(mapStateSelectStartPoint, mapDispatchSelectStartPoint)(SelectStartPointComponent);


class SelectLaneComponent extends React.Component {

  confirm() {
    this.props.scheduleEditorActions.setActiveStep(steps.selectLane.nextStep)
  }

  back() {
    this.props.scheduleEditorActions.backStep(steps.selectLane.previousStep)
  }

  getLaneList(laneList) {

    let resList = [];
    let key = 0;

    for (let lane of laneList) {
      if (laneList.hasOwnProperty(key)) {
        resList.push((
          <ListItem key={key}>
            <ListItemContent icon="navigation" key={key}>
              <strong>
                Lane ID: {lane}
              </strong>
            </ListItemContent>
          </ListItem>
        ));
        key += 1;
      }
    }
    return resList
  };

  render() {

    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px"}}>
        <CardHeader title="Select Lane"/>
        <CardText>
          <List>
            {this.getLaneList(this.props.laneList)}
          </List>
        </CardText>
        <CardActions border>
          <div style={{float: "right"}}>
            <Button onClick={this.back.bind(this)}>Back</Button>
            <Button onClick={this.confirm.bind(this)}>Confirm</Button>
          </div>
        </CardActions>
      </Card>
    )
  }
}

const mapStateSelectLane = (state) => ({
  activeStep: state.scheduleEditor.getActiveStep(),
  laneList: state.scheduleEditor.getLaneList()
});
const mapDispatchSelectLane = (dispatch) => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch),
});
let SelectLane = connect(mapStateSelectLane, mapDispatchSelectLane)(SelectLaneComponent);


class SelectEndPointComponent extends React.Component {

  confirm() {
    if (this.props.endPoint !== "") {
      this.props.scheduleEditorActions.setActiveStep(steps.selectEndPoint.nextStep)
    } else {
      alert("End point is not selected!");
    }
  }

  back() {
    this.props.scheduleEditorActions.backStep(steps.selectEndPoint.previousStep)
  }

  render() {
    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px"}}>
        <CardHeader>
          Select End Point
        </CardHeader>
        <CardText><strong>Selected Point ID: {this.props.endPoint}</strong></CardText>
        <CardActions border>
          <div style={{float: "right"}}>
            <Button onClick={this.back.bind(this)}>Back</Button>
            <Button onClick={this.confirm.bind(this)}>Confirm</Button>
          </div>
        </CardActions>
      </Card>
    )
  }
}

const mapStateSelectEndPoint = (state) => ({
  activeStep: state.scheduleEditor.getActiveStep(),
  endPoint: state.scheduleEditor.getEndPoint()
});
const mapDispatchSelectEndPoint = (dispatch) => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch),
});
let SelectEndPoint = connect(mapStateSelectEndPoint, mapDispatchSelectEndPoint)(SelectEndPointComponent);


class ResultComponent extends React.Component {

  reselect() {
    this.props.scheduleEditorActions.resetRouteCode()
  }

  back() {
    this.props.scheduleEditorActions.backStep(steps.result.previousStep)
  }

  getResult(startPoint, laneList, endPoint) {

    let lastPoint = "";
    let laneString = "";
    for (const lane of laneList) {
      const points = lane.split('_');

      const [arrow, startIndex, endIndex] = this.props.isBack ? ["<", 1, 0] : [">", 0, 1];

      laneString += points[startIndex] + arrow;
      lastPoint = points[endIndex];
    }
    laneString += lastPoint;

    return startPoint + ":" + laneString + ":" + endPoint
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
      errorMessages.push("Waypoints does not have this Start Point");
    }
    for (const laneID of laneList) {
      if (!lane.lanes.hasOwnProperty(laneID)) {
        isValidate = false;
        errorMessages.push("Lanes does not have this LaneID:" + laneID);
      } else {
        const index = laneList.indexOf(laneID);
        if (index === 0) {
          if (lane.lanes[laneID].waypointIDs.indexOf(startPoint) === -1) {
            isValidate = false;
            errorMessages.push("Start Point does not exist on the first lane");
          }
        } else {
          if (!this.props.isBack) {
            if (lane.toLanes[laneList[index - 1]].indexOf(laneList[index]) === -1) {
              isValidate = false;
              errorMessages.push("This Lane does not exist in toLanes of previous lane: LaneID is " + laneID);
            }
          } else {
            if (lane.fromLanes[laneList[index - 1]].indexOf(laneList[index]) === -1) {
              isValidate = false;
              errorMessages.push("This Lane does not exist in fromLanes of previous lane: LaneID is " + laneID);
            }
          }
        }
      }
    }

    if (lane.lanes.hasOwnProperty(laneList[laneList.length - 1])) {
      if (lane.lanes[laneList[laneList.length - 1]].waypointIDs.indexOf(endPoint) === -1) {
        isValidate = false;
        errorMessages.push("End Point does not exist on the last lane");
      }
    }

    if (laneList.length === 1 && lane.lanes.hasOwnProperty(laneList[0])) {
      const startIndex = lane.lanes[laneList[0]].waypointIDs.indexOf(startPoint);
      const endIndex = lane.lanes[laneList[0]].waypointIDs.indexOf(endPoint);

      if (!this.props.isBack && startIndex > endIndex) {
        isValidate = false;
        errorMessages.push("Start Point is behind End Point");
      } else if (this.props.isBack && startIndex < endIndex) {
        isValidate = false;
        errorMessages.push("Start Point is behind End Point");
      }
    }

    if (isValidate) {
      let res = [];
      res.push((<br/>));
      res.push((<p style={{color: "blue"}}>Validation is OK!</p>));
      return res
    } else {
      let errorList = [];
      errorList.push((<br/>));
      errorList.push((<p style={{color: "red"}}>Validation is Fail!</p>));
      for (const errorMessage of errorMessages) {
        errorList.push((<p style={{color: "red"}}>{errorMessage}</p>))
      }
      return errorList
    }
  }

  render() {
    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px"}}>
        <CardHeader>
          Result
        </CardHeader>
        <CardText>
          <div style={{wordWrap: "break-word"}}>
            <strong>Result: {this.getResult(this.props.startPoint, this.props.laneList, this.props.endPoint)}</strong>
            <br/>
            {this.validateResult()}
          </div>
        </CardText>
        <CardActions border>
          <div style={{float: "right"}}>
            <Button onClick={this.back.bind(this)}>Back</Button>
            <Button onClick={this.reselect.bind(this)}>Reselect</Button>
          </div>
        </CardActions>
      </Card>
    )
  }
}

const mapStateResult = (state) => ({
  activeStep: state.scheduleEditor.getActiveStep(),
  isBack: state.scheduleEditor.getIsBack(),
  startPoint: state.scheduleEditor.getStartPoint(),
  laneList: state.scheduleEditor.getLaneList(),
  endPoint: state.scheduleEditor.getEndPoint(),
  mapData: state.scheduleEditor.getMapData()
});
const mapDispatchResult = (dispatch) => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch),
});
let Result = connect(mapStateResult, mapDispatchResult)(ResultComponent);


class CreateRouteCodeTabs extends React.Component {

  constructor(props) {
    super(props);

    this.component = {
      advanceOrBack: {
        component: (<AdvanceOrBack/>),
      },
      selectStartPoint: {
        component: (<SelectStartPoint/>),
      },
      selectLane: {
        component: (<SelectLane/>),
      },
      selectEndPoint: {
        component: (<SelectEndPoint/>),
      },
      result: {
        component: (<Result/>),
      }
    };
  }

  getShowTab() {
    return this.component[this.props.activeStep].component
  }

  render() {
    return (
      <div>
        {this.getShowTab()}
      </div>

    );
  }
}

const mapState = (state) => ({
  activeStep: state.scheduleEditor.getActiveStep()
});

const mapDispatch = (dispatch) => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch),
});

export default connect(mapState, mapDispatch)(CreateRouteCodeTabs);