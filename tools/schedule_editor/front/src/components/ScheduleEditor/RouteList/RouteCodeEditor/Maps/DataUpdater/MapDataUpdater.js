import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { mapDataSelector } from '../../../../../../redux/selectors/ScheduleEditorSelector';

class MapDataUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.mapData !== undefined) {
      this.props.initMapData(this.props.mapData);
    }
  }

  componentDidUpdate() {
    if (this.props.mapData !== undefined) {
      this.props.setMapData(this.props.mapData);
    }
  }

  render() {
    return <div />;
  }
}
MapDataUpdater.propTypes = {
  mapData: PropTypes.object,
  initMapData: PropTypes.func,
  setMapData: PropTypes.func
};
const mapState = state => ({
  mapData: mapDataSelector(state)
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(MapDataUpdater);
