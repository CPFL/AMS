import React from 'react';
import {connect} from "react-redux";

import {mapDataSelector} from "../../../../redux/selectors/RouteCodeEditorSelector";

class WidthAndHeightUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidUpdate() {
    console.log(this.props);
    if (this.props.width > 0 && this.props.height > 0) {
      this.props.setWidthAndHeight(this.props.width, this.props.height);
    }
  }

  render() {
    return (<div/>)
  }
}

const mapState = (state) => ({
  width: state.routeCodeEditor.getWidth(),
  height: state.routeCodeEditor.getHeight()
});


const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(WidthAndHeightUpdater);
