import React from 'react';
import { connect } from 'react-redux';
import PropTypes from 'prop-types';

class IsBackUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidUpdate() {
    console.log(this.props);
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
