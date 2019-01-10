import React from 'react';
import PropTypes from 'prop-types';
import { bindActionCreators } from 'redux';
import connect from 'react-redux/es/connect/connect';

import CreateSchedule from './ScheduleEditSteps';
import ChangeRouteEditor from './ChangeRouteEditor/ChangeRouteEditor';

import * as ScheduleEditorActions from '../../../../redux/Actions/ScheduleEditorActions';

class ScheduleEditProcess extends React.Component {
  constructor(props) {
    super(props);

    this.component = {
      selectRouteCode: <CreateSchedule />,
      changeRouteEditor: <ChangeRouteEditor />
    };
  }

  getSteps() {
    return this.component[this.props.activeStepSchedule];
  }

  render() {
    const wrapper = {
      width: '100%',
      height: '100%'
    };

    return <div style={wrapper}>{this.getSteps()}</div>;
  }
}

ScheduleEditProcess.propTypes = {
  activeStepSchedule: PropTypes.string,
  scheduleEditorActions: PropTypes.object
};

const mapState = state => ({
  activeStepSchedule: state.scheduleEditor.getActiveStepScheduleEditor()
});
const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});

export default connect(
  mapState,
  mapDispatch
)(ScheduleEditProcess);
