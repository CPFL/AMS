import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { scheduleEditorActiveStepSelector } from '../../../../../../redux/selectors/ScheduleEditorSelector';

class ScheduleEditorActiveStepUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.scheduleEditorActiveStep) {
      this.props.initScheduleEditorActiveStep(
        this.props.scheduleEditorActiveStep
      );
    }
  }

  componentDidUpdate() {
    console.log(this.props);
    if (this.props.scheduleEditorActiveStep) {
      this.props.setScheduleEditorActiveStep(
        this.props.scheduleEditorActiveStep
      );
    }
  }

  render() {
    return <div />;
  }
}
ScheduleEditorActiveStepUpdater.propTypes = {
  scheduleEditorActiveStep: PropTypes.object,
  initScheduleEditorActiveStep: PropTypes.func,
  setScheduleEditorActiveStep: PropTypes.func
};

const mapState = state => ({
  scheduleEditorActiveStep: scheduleEditorActiveStepSelector(state)
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(ScheduleEditorActiveStepUpdater);
