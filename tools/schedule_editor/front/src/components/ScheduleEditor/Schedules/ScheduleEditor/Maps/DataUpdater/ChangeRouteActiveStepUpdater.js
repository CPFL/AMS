import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { changeRouteActiveStepSelector } from '../../../../../../redux/selectors/ScheduleEditorSelector';

class ChangeRouteActiveStepUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.changeRouteActiveStep >= 0) {
      this.props.setChangeRouteActiveStep(this.props.changeRouteActiveStep);
    }
  }

  componentDidUpdate() {
    console.log(this.props);
    if (this.props.changeRouteActiveStep >= 0) {
      this.props.setChangeRouteActiveStep(this.props.changeRouteActiveStep);
    }
  }

  render() {
    return <div />;
  }
}
ChangeRouteActiveStepUpdater.propTypes = {
  changeRouteActiveStep: PropTypes.number,
  setChangeRouteActiveStep: PropTypes.func
};

const mapState = state => ({
  changeRouteActiveStep: changeRouteActiveStepSelector(state)
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(ChangeRouteActiveStepUpdater);
