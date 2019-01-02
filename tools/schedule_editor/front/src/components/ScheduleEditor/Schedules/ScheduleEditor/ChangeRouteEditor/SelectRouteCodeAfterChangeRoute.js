import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';
import { bindActionCreators } from 'redux';

import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import CardActions from '@material-ui/core/CardActions';
import Button from '@material-ui/core/Button';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import Typography from '@material-ui/core/Typography';

import * as ScheduleEditorActions from '../../../../../redux/Actions/ScheduleEditorActions';
import { steps } from '../../../../../model/Redux/Page/ScheduleEditor';
import ListItemText from '@material-ui/core/ListItemText/ListItemText';
import InputLabel from "@material-ui/core/InputLabel/InputLabel";
import Select from "@material-ui/core/Select/Select";
import MenuItem from "@material-ui/core/MenuItem/MenuItem";

class SelectRouteCodeAfterChangeRoute extends React.Component {
  constructor(props) {
    super(props);
    this.confirm = this.confirm.bind(this);
  }

  confirm() {
    if (this.props.changeRouteStartPoint !== '') {
      this.props.scheduleEditorActions.setChangeRouteActiveStepNext(
        this.props.changeRouteActiveStep
      );
    } else {
      alert('Start point is not selected!');
    }
  }

  setSelectRouteCode(event) {
    const [currentRouteCode] = this.props.selectableRouteCode.filter(
      routeCode => routeCode.routeCode === event.target.value
    );
    this.props.scheduleEditorActions.setCurrentRouteCodeSchedule(
      currentRouteCode
    );
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
    return (
      <Card shadow={0} style={{ width: '100%', minHeight: '100px' }}>
        <CardHeader title="Select Start Point" />
        <CardContent>
          <InputLabel htmlFor="route-code">Route Code</InputLabel>
          <Select
            value={this.props.routeCodeAfterChangeRoute.routeCode}
            onChange={this.setSelectRouteCode}
            inputProps={{
              name: 'route-code'
            }}
            style={{ marginLeft: '5px' }}
          >
            {this.getSelectItem()}
          </Select>
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

SelectRouteCodeAfterChangeRoute.propTypes = {
  changeRouteActiveStep: PropTypes.string,
  routeCodeAfterChangeRoute: PropTypes.object,
  currentRouteCodeSchedule: PropTypes.object,
  routeCodeList: PropTypes.array,
  scheduleEditorActions: PropTypes.object
};

const mapState = state => ({
  changeRouteActiveStep: state.scheduleEditor.getChangeRouteActiveStep(),
  routeCodeAfterChangeRoute: state.getRouteCodeAfterChangeRoute(),
  currentRouteCodeSchedule: state.getCurrentRouteCodeSchedule(),
  routeCodeList: state.getRouteCodeList()
});
const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapState,
  mapDispatch
)(SelectRouteCodeAfterChangeRoute);
