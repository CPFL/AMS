import React from 'react';
import {connect} from "react-redux";

import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import Button from '@material-ui/core/Button';
import Dialog from '@material-ui/core/Dialog';
import ScheduleList from './ScheduleList';

class ScheduleBox extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      listHeight: 0,
      isOpen: false
    };

    this.addScheduleModalOpen = this.addScheduleModalOpen.bind(this);
    this.closeModal = this.closeModal.bind(this);
    this.resize = this.resize.bind(this);
  }

  componentDidMount() {
    this.setState({listHeight: document.getElementById("ScheduleCard").clientHeight - document.getElementById("ScheduleCardHeader").clientHeight});
    document.getElementById("ScheduleCard").addEventListener('resize', this.resize, false);
  }

  resize() {
    this.setState({listHeight: document.getElementById("ScheduleCard").clientHeight - document.getElementById("ScheduleCardHeader").clientHeight});
  }

  addScheduleModalOpen(){
    this.setState({isOpen: true});
  }

  closeModal(){
    this.setState({isOpen: false});
  }

  render() {
    const cardContentHeight = this.state.listHeight;

    const wrapper = {
      padding: '5px',
      boxSizing: 'border-box',
      height: '100%'
    };

    const contentStyle = {
      boxSizing: 'border-box',
      height: cardContentHeight
    };

    return (
      <div style={wrapper}>
        <Card style={{height: '100%'}}
              id="ScheduleCard"
        >
          <CardHeader
            action={
              <Button variant="outlined" onClick={this.addScheduleModalOpen}>
                Add Schedule
              </Button>
            }
            title="Schedule"
            id="ScheduleCardHeader"
          />
          <CardContent style={contentStyle}>
            <ScheduleList/>
          </CardContent>
        </Card>
        <Dialog
          open={this.state.isOpen}
          onClose={this.closeModal}
        >
          Test
        </Dialog>
      </div>

    );
  }
}

const mapState = () => ({});

const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(ScheduleBox);

