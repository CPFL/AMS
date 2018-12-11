import React from 'react';
import connect from "react-redux/es/connect/connect";

import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import DoneIcon from '@material-ui/icons/Done';

class RouteList extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      listHeight: 0
    };
    this.selectRouteCode.bind(this);
  }

  getItems() {
    const routeCodeList = this.props.routeCodeList;
    const resList = [];
    for (const item of routeCodeList) {
      resList.push(
        <ListItem button onClick={event => this.selectRouteCode(event, item)}>
          <ListItemIcon>
            <DoneIcon/>
          </ListItemIcon>
          <ListItemText primary={
            <div style={{wordBreak: 'break-all'}}>
              {item}
            </div>
          }/>
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

const mapStateSelectEndPoint = (state) => ({
  routeCodeList: state.scheduleEditor.getRouteCodeList(),
});
const mapDispatchSelectEndPoint = () => ({});
export default connect(mapStateSelectEndPoint, mapDispatchSelectEndPoint)(RouteList);
