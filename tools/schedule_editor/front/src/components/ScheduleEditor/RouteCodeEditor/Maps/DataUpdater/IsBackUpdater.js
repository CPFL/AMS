import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

class IsBackUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidUpdate() {
    if (this.props.isBack !== undefined) {
      this.props.setIsBack(this.props.isBack);
    }
  }

  render() {
    return <div />;
  }
}
IsBackUpdater.propTypes = {
  isBack: PropTypes.bool,
  setIsBack: PropTypes.func
};
const mapState = state => ({
  isBack: state.scheduleEditor.getIsBack()
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(IsBackUpdater);
