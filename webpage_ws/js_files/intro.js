'use strict'
const e = React.createElement;
let ros = null

class IntroComponent extends React.Component {
    constructor(props) {
        super(props)
        this.state = {
            connected: false
        }
    }

    connectToRosbridge = () => {
        try {
            ros = new ROSLIB.Ros({
                url: document.getElementById("rosbridge_address").value
            })
        } catch (ex) {
            console.log(ex)
        }

        ros.on('connection', function () {
            console.log('Connected to websocket server.')
            this.setState({ connected: true })
            window.location.href = "../display.html"
            // this.robotMoveTest()
        })

        ros.on('error', function (error) {
            console.log('Error connecting to websocket server: ', error)
        })
        ros.on('close', function () {
            console.log('Connection to websocket server closed.')
            this.setState({ connected: false })
        })
    }

    // for testing... something is wrong here.
    robotMoveTest = () => {
        let topic = new ROSLIB.Topic({
            ros: ros,
            name: '/robot/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        })
        let msg = new ROSLIB.Message({
            linear: { x: 0.25 },
            angular: { z: 0.05 },
        })
        topic.publish(msg)
        this.robotState = 'running in circles...'
    }

    render() {
        return (
            <div id="full-screen-container">
                <div id="login-container">
                    <h1 id="login-title"> RGBD SLAM WebServer </h1>
                    <form id="form">
                        <div id="input-group">
                            <label> Rosbridge Address: </label>
                            <input type="text" name="rosbridge_address" id="rosbridge_address" />
                            <span id="msg_error"> </span>
                        </div>
                        <button id="login-button" onClick={this.connectToRosbridge}>Connect</button>
                    </form>
                </div>
            </div>
        )
    }
}

ReactDOM.render(e(IntroComponent), document.querySelector('#intro'))