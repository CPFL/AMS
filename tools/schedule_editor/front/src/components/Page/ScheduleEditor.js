import React from 'react';

export default class ScheduleEditor extends React.Component {

  constructor(props) {
    super(props);
  }

  render() {

    let wrapper = {
      position: 'absolute',
      top: 0,
      right: 0,
      bottom: 0,
      left: 0
    };

    return (
      <div style={wrapper}>
        Hello World!
      </div>

    );
  }
}
