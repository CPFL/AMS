import React from 'react';
import {connect} from "react-redux";

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
    console.log(this.props);
    if (this.props.activeStep !== undefined) {
      this.props.setActiveStep(this.props.activeStep);
    }
  }

  render() {
    return (<div/>)
  }
}

const mapState = (state) => ({
  activeStep: state.scheduleEditor.getActiveStep()
});


const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(ActiveStepUpdater);
