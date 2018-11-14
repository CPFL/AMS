import React from 'react';
import {connect} from "react-redux";

class WaypointsUpdater extends React.Component {
    constructor(props) {
        super(props);
    }

    componentDidUpdate() {
        if(this.props.waypoints !== undefined) {
            this.props.setWaypoints(this.props.waypoints);
        }
    }

    render() {
        return (<div/>)
    }
}

const mapState = (state) => ({
        waypoints: state.operatingMapFromLocal.getWaypoints()
});

const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(WaypointsUpdater);
