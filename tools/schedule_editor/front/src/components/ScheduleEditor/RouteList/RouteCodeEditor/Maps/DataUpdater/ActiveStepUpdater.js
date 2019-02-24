import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

class ActiveStepUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.activeStep !== undefined) {
      this.props.setActiveStep(this.props.activeStep);
    }
  }

  componentDidUpdate() {
    if (this.props.activeStep !== undefined) {
      this.props.setActiveStep(this.props.activeStep);
    }
  }

  render() {
    return <div />;
  }
}

ActiveStepUpdater.propTypes = {
  activeStep: PropTypes.string,
  setActiveStep: PropTypes.func
};
const mapState = state => ({
  activeStep: state.scheduleEditor.getActiveStep()
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(ActiveStepUpdater);
