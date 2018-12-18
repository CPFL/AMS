import React from 'react';
import { bindActionCreators } from 'redux';
import connect from 'react-redux/es/connect/connect';

import Card from '@material-ui/core/Card/Card';
import CardHeader from '@material-ui/core/CardHeader/CardHeader';
import CardContent from '@material-ui/core/CardContent/CardContent';
import Select from '@material-ui/core/Select';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import Checkbox from '@material-ui/core/Checkbox';
import InputLabel from '@material-ui/core/InputLabel';
import TextField from '@material-ui/core/TextField';
import CardActions from '@material-ui/core/CardActions/CardActions';
import MenuItem from '@material-ui/core/MenuItem';
import Button from '@material-ui/core/Button/Button';
import AddIcon from '@material-ui/icons/Add';
import Typography from '@material-ui/core/Typography';

import PropTypes from 'prop-types';
import { scheduleEditSteps } from '../../../../model/Redux/Page/ScheduleEditor';
import * as ScheduleEditorActions from '../../../../redux/Actions/ScheduleEditorActions';

import ChangeRouteList from './ChangeRouteList';

class SelectRouteCodeComponent extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      routeCode: '',
      checkedSendEngage: false,
      waitTime: 0,
      addScheduleCardHegiht: 0
    };

    this.resize = this.resize.bind(this);
    this.setSelectRouteCode = this.setSelectRouteCode.bind(this);
    this.setIsSendEngage = this.setIsSendEngage.bind(this);
    this.setWaitTime = this.setWaitTime.bind(this);
  }

  componentDidMount() {
    this.setState({
      addScheduleCardHegiht:
        document.getElementById('AddScheduleCard').clientHeight -
        document.getElementById('AddScheduleCardActions').clientHeight
    });

    //document.getElementById('AddScheduleCard').addEventListener('resize', resize, false);
    window.addEventListener('resize', this.resize, false);
  }

  resize() {
    this.setState({
      addScheduleCardHegiht:
        document.getElementById('AddScheduleCard').clientHeight -
        document.getElementById('AddScheduleCardActions').clientHeight
    });
  }

  componentWillUnmount() {
    document
      .getElementById('AddScheduleCard')
      .removeEventListener('resize', this.resize, false);
  }

  setSelectRouteCode(event) {
    this.setState({ routeCode: event.target.value });
  }

  setIsSendEngage(event) {
    console.log(event);
    this.setState({ checkedSendEngage: event.target.checked });
  }

  setWaitTime(event) {
    console.log(event);
    this.setState({ waitTime: event.target.value });
  }

  getNextStepButtons() {
    const resList = [];
    const nextStepList = scheduleEditSteps.selectRouteCode.nextStep;
    for (const nextStep of nextStepList) {
      const confirm = () => {
        this.props.scheduleEditorActions.setActiveStepScheduleEditor(nextStep);
      };
      resList.push(
        <Button color="primary" onClick={confirm}>
          {nextStep}
        </Button>
      );
    }

    return resList;
  }

  render() {
    const selectableRouteCodeList = this.props.selectableRouteCode;
    const addScheduleCardHegiht = this.state.addScheduleCardHegiht;

    console.log(addScheduleCardHegiht);

    const getSelectItem = () => {
      const resList = [];

      resList.push(
        <MenuItem value="">
          <em>None</em>
        </MenuItem>
      );

      for (const routeCode of selectableRouteCodeList) {
        resList.push(
          <MenuItem value={routeCode}>
            <em>{routeCode}</em>
          </MenuItem>
        );
      }
      return resList;
    };

    const addScheduleCardStyle = {
      width: '100%',
      height: '100%'
    };

    const addScheduleCardContentStyle = {
      boxSizing: 'border-box',
      height: addScheduleCardHegiht,
      overflowY: 'auto'
    };

    const changeRouteListCardStyle = {
      height: '200px',
      marginTop: '24px'
    };

    const changeRouteListCardContentStyle = {
      boxSizing: 'border-box',
      height: '150px'
    };

    return (
      <Card style={addScheduleCardStyle} id="AddScheduleCard">
        <CardContent style={addScheduleCardContentStyle}>
          <InputLabel htmlFor="route-code">Route Code</InputLabel>
          <Select
            value={this.state.routeCode}
            onChange={this.setSelectRouteCode}
            inputProps={{
              name: 'route-code'
            }}
          >
            {getSelectItem()}
          </Select>
          <br />
          <div style={{ marginTop: '10px', position: 'relative' }}>
            <FormControlLabel
              control={
                <Checkbox
                  checked={this.state.checkedSendEngage}
                  onChange={this.setIsSendEngage}
                  value="sendEngage"
                  color="primary"
                />
              }
              label="Send Engage"
              style={{ marginTop: '24px' }}
            />
            <TextField
              id="wait-time"
              label="Wait Time"
              value={this.state.waitTime}
              onChange={this.setWaitTime}
              style={{ width: '100px' }}
              type="number"
              InputLabelProps={{
                shrink: true
              }}
              inputProps={{
                min: '0',
                step: '1'
              }}
              margin="normal"
            />
          </div>
          <Card style={changeRouteListCardStyle}>
            <CardHeader
              action={
                <Button variant="contained" color="primary" size="small">
                  <AddIcon
                    style={{ color: 'white', marginRight: '5px' }}
                    fontSize="small"
                  />
                  Add Change Route
                </Button>
              }
              title={
                <Typography variant="subtitle1" gutterBottom>
                  Change Route List
                </Typography>
              }
            />
            <CardContent style={changeRouteListCardContentStyle}>
              <ChangeRouteList />
            </CardContent>
          </Card>
        </CardContent>
        <CardActions id="AddScheduleCardActions">
          <div style={{ marginLeft: 'auto' }}>{this.getNextStepButtons()}</div>
        </CardActions>
      </Card>
    );
  }
}
SelectRouteCodeComponent.propTypes = {
  activeStepSchedule: PropTypes.string,
  selectableRouteCode: PropTypes.array,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelectRouteCode = state => ({
  activeStepSchedule: state.scheduleEditor.getActiveStepScheduleEditor(),
  selectableRouteCode: state.scheduleEditor.getSelectableRouteCode()
});
const mapDispatchSelectRouteCode = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export const SelectRouteCode = connect(
  mapStateSelectRouteCode,
  mapDispatchSelectRouteCode
)(SelectRouteCodeComponent);

class ResultComponent extends React.Component {
  constructor(props) {
    super(props);

    this.back = this.back.bind(this);
    this.reselect = this.reselect.bind(this);
  }

  reselect() {
    //this.props.scheduleEditorActions.resetRouteCode();
  }

  back() {
    //this.props.scheduleEditorActions.backStep(steps.result.previousStep);
  }

  render() {
    return (
      <Card style={{ width: '100%', minHeight: '100px' }}>
        <CardHeader title="Result" />
        <CardContent>Test</CardContent>
        <CardActions>
          <div style={{ marginLeft: 'auto' }}>
            <Button variant="outlined" onClick={this.back}>
              Back
            </Button>
            <Button
              variant="outlined"
              color="primary"
              onClick={() => {
                //this.save(routeCode);
              }}
              style={{ marginLeft: '5px' }}
            >
              Save
            </Button>
            <Button
              variant="outlined"
              color="primary"
              onClick={() => {
                //this.saveAndAnotherSelect(routeCode);
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
  activeStepSchedule: PropTypes.string,
  scheduleEditorActions: PropTypes.object
};
const mapStateResult = state => ({
  activeStepSchedule: state.scheduleEditor.getActiveStepScheduleEditor()
});
const mapDispatchResult = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});

export const Result = connect(
  mapStateResult,
  mapDispatchResult
)(ResultComponent);
