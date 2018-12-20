import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { currentRouteCodeScheduleSelector } from '../../../../../../redux/selectors/ScheduleEditorSelector';

class RouteCodeUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    const currentRouteCodeSchedule = this.props.currentRouteCodeSchedule;
    console.log(currentRouteCodeSchedule);
    if (currentRouteCodeSchedule) {
      this.props.initRouteCode(currentRouteCodeSchedule);
    }
  }

  componentDidUpdate() {
    const currentRouteCodeSchedule = this.props.currentRouteCodeSchedule;
    console.log(currentRouteCodeSchedule);
    if (currentRouteCodeSchedule) {
      this.props.updateRouteCode(currentRouteCodeSchedule);
    }
  }

  render() {
    return <div />;
  }
}
RouteCodeUpdater.propTypes = {
  currentRouteCodeSchedule: PropTypes.object,
  initRouteCode: PropTypes.func,
  updateRouteCode: PropTypes.func
};

const mapState = state => ({
  currentRouteCodeSchedule: currentRouteCodeScheduleSelector(state)
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(RouteCodeUpdater);
