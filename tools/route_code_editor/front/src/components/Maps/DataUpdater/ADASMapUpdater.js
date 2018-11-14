import React from 'react';
import {connect} from "react-redux";

class ADASMapUpdater extends React.Component {
    constructor(props) {
        super(props);
    }

    componentDidUpdate() {
        if(this.props.adasmap !== undefined) {
            this.props.setADASMap(this.props.adasmap);
        }
    }

    render() {
        return (<div/>)
    }
}

const mapState = (state) => ({
    adasmap: state.operatingMapFromLocal.getAdasmap()
});

const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(ADASMapUpdater);
