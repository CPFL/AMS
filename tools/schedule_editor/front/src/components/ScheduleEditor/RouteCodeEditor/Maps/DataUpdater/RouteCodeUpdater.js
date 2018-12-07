import React from 'react';
import {connect} from "react-redux";

import {routeCodeSelector} from '../../../../../redux/selectors/ScheduleEditorSelector';

class RouteCodeUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidUpdate() {
    console.log(this.props);
    if (this.props.routeCode.startPoint != null && this.props.routeCode.laneList != null && this.props.routeCode.endPoint != null) {
      this.props.updateRouteCode(
        this.props.routeCode.startPoint,
        this.props.routeCode.laneList,
        this.props.routeCode.endPoint
      );
    }
  }

  render() {
    return (<div/>)
  }
}

const mapState = (state) => ({
  routeCode: routeCodeSelector(state),
});


const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(RouteCodeUpdater);
