import React from 'react';
import {connect} from "react-redux";

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
    return (<div/>)
  }
}

const mapState = (state) => ({
  isBack: state.scheduleEditor.getIsBack()
});


const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(IsBackUpdater);
