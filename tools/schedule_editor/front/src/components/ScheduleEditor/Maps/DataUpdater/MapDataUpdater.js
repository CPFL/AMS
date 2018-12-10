import React from 'react';
import {connect} from "react-redux";

import {mapDataSelector} from "../../../../redux/selectors/ScheduleEditorSelector";

class MapDataUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidUpdate() {
    console.log(this.props);
    if (this.props.mapData !== undefined) {
      this.props.setMapData(this.props.mapData);
    }
  }

  render() {
    return (<div/>)
  }
}

const mapState = (state) => ({
  mapData: mapDataSelector(state)
});

const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(MapDataUpdater);
