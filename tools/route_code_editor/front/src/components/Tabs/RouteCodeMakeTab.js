import React from 'react';


import {
  Card,
  CardTitle,
  CardText,
  CardActions,
  Button,
  Switch,
  List,
  ListItem,
  ListItemContent,
} from 'react-mdl';

import AppBar from '@material-ui/core/AppBar';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';

import {connect} from "react-redux";
import * as RouteCodeEditorActions from "../../redux/Actions/RouteCodeEditorActions";
import {bindActionCreators} from "redux";

import {steps} from '../../model/Redux/Page/RouteCodeEditor'

class AdvanceOrBackComponent extends React.Component {

  confirm(){
    this.props.routeCodeEditorActions.setActiveStep("selectStartPoint")
  }

  render() {
    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px", marginTop: "10px"}}>
        <CardTitle>
          Select Advance or Back
        </CardTitle>
        <CardActions border>
          <Switch ripple>Back</Switch>
          <div style={{float: "right"}}>
            <Button onClick={this.confirm.bind(this)}>Confirm</Button>
          </div>
        </CardActions>
      </Card>
    )
  }
}

const mapStateAdvanceOrBack = (state) => ({
  activeStep: state.routeCodeEditor.getActiveStep()
});

const mapDispatchAdvanceOrBack = (dispatch) => ({
  routeCodeEditorActions: bindActionCreators(RouteCodeEditorActions, dispatch),
});

let AdvanceOrBack = connect(mapStateAdvanceOrBack, mapDispatchAdvanceOrBack)(AdvanceOrBackComponent);


class SelectStartPointComponent extends React.Component {

  confirm(){
    this.props.routeCodeEditorActions.setActiveStep("selectLane")
  }

  render() {
    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px", marginTop: "10px"}}>
        <CardTitle>
          Select Start Point
        </CardTitle>
        <CardText><strong>Selected Point ID: </strong></CardText>
        <CardActions border>
          <div style={{float: "right"}}>
            <Button onClick={this.confirm.bind(this)}>Confirm</Button>
          </div>
        </CardActions>
      </Card>

    )
  }
}

const mapStateSelectStartPoint = (state) => ({
  activeStep: state.routeCodeEditor.getActiveStep()
});
const mapDispatchSelectStartPoint = (dispatch) => ({
  routeCodeEditorActions: bindActionCreators(RouteCodeEditorActions, dispatch),
});
let SelectStartPoint = connect(mapStateSelectStartPoint, mapDispatchSelectStartPoint)(SelectStartPointComponent);


class SelectLaneComponent extends React.Component {

  confirm(){
    this.props.routeCodeEditorActions.setActiveStep("selectEndPoint")
  }

  getLaneList() {

    let resList = [];
    let laneList = this.props.LaneList;
    laneList = {
      a: 1,
      b: 2
    };

    for (let key in laneList) {

      if (laneList.hasOwnProperty(key)) {
        resList.push((
          <ListItem key={key} twoLine>
            <ListItemContent icon="navigation" key={key}>
              <strong>
                Start Points ID: 1
              </strong>
              <br/>
              <strong>
                End Points ID: 3
              </strong>
            </ListItemContent>
          </ListItem>
        ));
      }
    }
    return resList
  };

  render() {
    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px", marginTop: "10px"}}>
        <CardTitle>
          Select Lane
        </CardTitle>
        <CardText>
          <List>
            {this.getLaneList()}
          </List>
        </CardText>
        <CardActions border>
          <div style={{float: "right"}}>
            <Button onClick={this.confirm.bind(this)}>Confirm</Button>
          </div>
        </CardActions>
      </Card>
    )
  }
}
const mapStateSelectLane = (state) => ({
  activeStep: state.routeCodeEditor.getActiveStep()
});
const mapDispatchSelectLane = (dispatch) => ({
  routeCodeEditorActions: bindActionCreators(RouteCodeEditorActions, dispatch),
});
let SelectLane = connect(mapStateSelectLane, mapDispatchSelectLane)(SelectLaneComponent);


class SelectEndPointComponent extends React.Component {

  confirm(){
    this.props.routeCodeEditorActions.setActiveStep("result")
  }

  render() {
    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px", marginTop: "10px"}}>
        <CardTitle>
          Select End Point
        </CardTitle>
        <CardText><strong>Selected Point ID: </strong></CardText>
        <CardActions border>
          <div style={{float: "right"}}>
            <Button onClick={this.confirm.bind(this)}>Confirm</Button>
          </div>
        </CardActions>
      </Card>
    )
  }
}
const mapStateSelectEndPoint = (state) => ({
  activeStep: state.routeCodeEditor.getActiveStep()
});
const mapDispatchSelectEndPoint = (dispatch) => ({
  routeCodeEditorActions: bindActionCreators(RouteCodeEditorActions, dispatch),
});
let SelectEndPoint = connect(mapStateSelectEndPoint, mapDispatchSelectEndPoint)(SelectEndPointComponent);


class ResultComponent extends React.Component {
  render() {
    return (
      <Card shadow={0}
            style={{width: "100%", minHeight: "100px", marginTop: "10px"}}>
        <CardTitle>
          Result
        </CardTitle>
        <CardText><strong>Result: </strong></CardText>
        <CardActions border>
          <div style={{float: "right"}}>
            <Button>Reselect</Button>
          </div>
        </CardActions>
      </Card>

    )
  }
}
const mapStateResult = (state) => ({
  activeStep: state.routeCodeEditor.getActiveStep()
});
const mapDispatchResult = (dispatch) => ({
  routeCodeEditorActions: bindActionCreators(RouteCodeEditorActions, dispatch),
});
let Result = connect(mapStateResult, mapDispatchResult)(ResultComponent);




class RouteCodeEditor extends React.Component {

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
      },
    };
  }

  setActiveTab(event, value) {
    //this.props.routeCodeEditorActions.setActiveStep(value)
  }

  getShowTab() {
    return this.component[this.props.activeStep].component
  }

  render() {

    return (
      <div>
        <AppBar position="static" color="default">
          <Tabs value={this.props.activeStep} onChange={this.setActiveTab.bind(this)} scrollable>
            <Tab value={steps.advanceOrBack.id} label={steps.advanceOrBack.name}/>
            <Tab value={steps.selectStartPoint.id} label={steps.selectStartPoint.name}/>
            <Tab value={steps.selectLane.id} label={steps.selectLane.name}/>
            <Tab value={steps.selectEndPoint.id} label={steps.selectEndPoint.name}/>
            <Tab value={steps.result.id} label={steps.result.name}/>
          </Tabs>
        </AppBar>
        {this.getShowTab()}

      </div>

    );
  }
}

const mapState = (state) => ({
  activeStep: state.routeCodeEditor.getActiveStep()
});

const mapDispatch = (dispatch) => ({
  routeCodeEditorActions: bindActionCreators(RouteCodeEditorActions, dispatch),
});

export default connect(mapState, mapDispatch)(RouteCodeEditor);