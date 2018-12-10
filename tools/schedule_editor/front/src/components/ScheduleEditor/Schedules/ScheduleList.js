import React from 'react';
import {connect} from "react-redux";

import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import DoneIcon from '@material-ui/icons/Done';


class ScheduleList extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      listHeight: 0
    };
  }

  render() {
    const routeListStyle = {
      overflowY: 'auto',
      height: '100%'
    };

    return (
      <List style={routeListStyle}>
        Test
      </List>
    );
  }
}

const mapState = () => ({});

const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(ScheduleList);

