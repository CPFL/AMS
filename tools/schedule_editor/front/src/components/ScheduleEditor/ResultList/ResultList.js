import React from 'react';
import {connect} from "react-redux";

import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import DoneIcon from '@material-ui/icons/Done';


class ResultList extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      listHeight: 0
    };
    this.selectRouteCode.bind(this);
  }

  getItems() {
    const testList = [

      {
        name: "send_lane_array",
        routeCode: "55:54>67>130:129",
        engageType: "",
        changeLane: [
          "68:67>130:120",
          "68:67>130>55>300>256:244"
        ]
      },
      "55:54>67>130>65:77",
      "55:54>67>130>65>500:499"
    ];
    let resList = [];
    for (const item of testList) {

      resList.push(
        <ListItem button onClick={event => this.selectRouteCode(event, item)}>
          <ListItemIcon>
            <DoneIcon/>
          </ListItemIcon>
          <ListItemText
            primary={'Lane:' + item.routeCode }
            secondary={}
          />
        </ListItem>
      )
    }
    return resList
  }

  selectRouteCode(event, item) {
    console.log(item);
  }

  render() {
    const routeListStyle = {
      overflowY: 'auto',
      height: '100%'
    };

    return (
      <List style={routeListStyle}>
        {this.getItems()}
      </List>
    );
  }
}

const mapState = () => ({});

const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(ResultList);

