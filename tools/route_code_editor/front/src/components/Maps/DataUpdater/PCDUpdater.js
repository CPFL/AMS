import React from 'react';
import {connect} from "react-redux";

class LoadPCDsUpdater extends React.Component {
    constructor(props) {
        super(props);
    }

    componentDidUpdate() {
        if(this.props.pcdList !== undefined) {
            this.props.setPCD(this.props.pcdList);
        }
    }

    render() {
        return (<div/>)
    }
}

const mapState = (state) => ({
    pcdList: state.operatingMapFromLocal.getPCD()
});

const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(LoadPCDsUpdater);
