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

    this.setSelectRouteCode = this.setSelectRouteCode.bind(this);
    this.setIsSendEngage = this.setIsSendEngage.bind(this);
    this.setWaitTime = this.setWaitTime.bind(this);
  }

  setSelectRouteCode(event) {
    const [currentRouteCode] = this.props.selectableRouteCode.filter(
      routeCode => routeCode.routeCode === event.target.value
    );
    this.props.scheduleEditorActions.setCurrentRouteCodeSchedule(
      currentRouteCode
    );
  }

  setIsSendEngage(event) {
    this.props.scheduleEditorActions.setCheckedSendEngage(event.target.checked);
  }

  setWaitTime(event) {
    this.props.scheduleEditorActions.setWaitTime(event.target.value);
  }

  getSelectItem() {
    const resList = [];

    resList.push(
      <MenuItem value="">
        <em>None</em>
      </MenuItem>
    );

    for (const routeCode of this.props.selectableRouteCode) {
      resList.push(
        <MenuItem value={routeCode.routeCode}>
          <em>{routeCode.routeCode}</em>
        </MenuItem>
      );
    }
    return resList;
  }

  render() {
    const test = {
      endPoint: '8878',
      isBack: false,
      laneList: ['8805_8855', '8855_8871', '8871_8873', '8873_8883'],
      routeCode: '8829:8805>8855>8871>8873>8883:8878',
      startPoint: '8829'
    };
    this.props.selectableRouteCode.push(test);

    const wrapper = {
      paddingTop: '5px',
      paddingLeft: '5px',
      paddingBottom: '5px',
      height: '100%',
      boxSizing: 'border-box'
    };

    const addScheduleCardStyle = {
      width: '100%',
      height: '100%'
    };

    const addScheduleCardContentStyle = {
      boxSizing: 'border-box',
      height: 'calc(100% - 52px)',
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
      <div style={wrapper}>
        <Card style={addScheduleCardStyle}>
          <CardContent style={addScheduleCardContentStyle}>
            <InputLabel htmlFor="route-code">Route Code</InputLabel>
            <Select
              value={this.props.currentRouteCodeSchedule.routeCode}
              onChange={this.setSelectRouteCode}
              inputProps={{
                name: 'route-code'
              }}
              style={{ marginLeft: '5px' }}
            >
              {this.getSelectItem()}
            </Select>
            <br />
            <div style={{ marginTop: '10px', position: 'relative' }}>
              <FormControlLabel
                control={
                  <Checkbox
                    checked={this.props.checkedSendEngage}
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
                value={this.props.waitTime}
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
                  <Button
                    variant="contained"
                    color="primary"
                    size="small"
                    onClick={() => {
                      this.props.scheduleEditorActions.setActiveStepScheduleEditor(
                        'changeRouteEditor'
                      );
                    }}
                  >
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
          <CardActions>
            <div style={{ marginLeft: 'auto' }}>
              <Button
                color="primary"
                onClick={() => console.log('save schedule')}
              >
                Save
              </Button>
            </div>
          </CardActions>
        </Card>
      </div>
    );
  }
}
SelectRouteCodeComponent.propTypes = {
  activeStepSchedule: PropTypes.string,
  selectableRouteCode: PropTypes.array,
  currentRouteCodeSchedule: PropTypes.object,
  checkedSendEngage: PropTypes.bool,
  waitTime: PropTypes.number,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelectRouteCode = state => ({
  activeStepSchedule: state.scheduleEditor.getActiveStepScheduleEditor(),
  selectableRouteCode: state.scheduleEditor.getSelectableRouteCode(),
  currentRouteCodeSchedule: state.scheduleEditor.getCurrentRouteCodeSchedule(),
  checkedSendEngage: state.scheduleEditor.getCheckedSendEngage(),
  waitTime: state.scheduleEditor.getWaitTime()
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
