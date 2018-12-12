import React from 'react';
import { connect } from 'react-redux';
import PropTypes from 'prop-types';

class ActiveStepUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidUpdate() {
    console.log(this.props);
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
