import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { routeCodeAfterChangeRouteSelector } from '../../../../../../redux/selectors/ScheduleEditorSelector';

class RouteCodeAfterChangeRouteUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.routeCodeAfterChangeRoute) {
      this.props.initRouteCodeAfterChangeRoute(
        this.props.routeCodeAfterChangeRoute
      );
    }
  }

  componentDidUpdate() {
    if (this.props.routeCodeAfterChangeRoute) {
      this.props.updateRouteCodeAfterChangeRoute(
        this.props.routeCodeAfterChangeRoute
      );
    }
  }

  render() {
    return <div />;
  }
}
RouteCodeAfterChangeRouteUpdater.propTypes = {
  routeCodeAfterChangeRoute: PropTypes.object,
  initRouteCodeAfterChangeRoute: PropTypes.func,
  updateRouteCodeAfterChangeRoute: PropTypes.func
};

const mapState = state => ({
  routeCodeAfterChangeRoute: routeCodeAfterChangeRouteSelector(state)
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(RouteCodeAfterChangeRouteUpdater);
