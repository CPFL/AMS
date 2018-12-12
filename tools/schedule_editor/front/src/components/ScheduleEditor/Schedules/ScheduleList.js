import React from 'react';

import List from '@material-ui/core/List';

export default class ScheduleList extends React.Component {
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

    return <List style={routeListStyle}>Test</List>;
  }
}
