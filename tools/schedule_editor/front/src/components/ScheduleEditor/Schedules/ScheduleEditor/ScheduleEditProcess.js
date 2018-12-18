import React from 'react';
import PropTypes from 'prop-types';
import { bindActionCreators } from 'redux';
import connect from 'react-redux/es/connect/connect';

import { SelectRouteCode, Result } from './ScheduleEditSteps';
import * as ScheduleEditorActions from '../../../../redux/Actions/ScheduleEditorActions';

class ScheduleEditProcess extends React.Component {
  constructor(props) {
    super(props);

    this.component = {
      selectRouteCode: <SelectRouteCode />,
      result: <Result />
      /*
      SelectChangeRoute: {
        component: <SelectChangeRoute />
      },
      result: {
        component: <Result />
      }
      */
    };
  }

  getSteps() {
    console.log(this.props.activeStepSchedule);
    console.log(this.component[this.props.activeStepSchedule]);

    return this.component[this.props.activeStepSchedule];
  }

  render() {
    const wrapper = {
      width: '100%',
      height: '100%'
    };

    const ProcessBoxStyle = {
      paddingTop: '5px',
      paddingLeft: '5px',
      paddingBottom: '5px',
      height: '100%',
      boxSizing: 'border-box'
    };
    return (
      <div style={wrapper}>
        <div style={ProcessBoxStyle}>{this.getSteps()}</div>
      </div>
    );
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
