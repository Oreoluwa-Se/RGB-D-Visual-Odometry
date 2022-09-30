'use strict'
const e = React.createElement;
let ros = null

class SubscribeComponent extends React.Component {
    constructor(props) {
        super(props)
        this.pingInterval = null

    }

    imageSubs = () => {
        var image_topic_1 = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/robot/front_rgbd_camera/rgb/image_raw/compressed',
            messageType: 'sensor_msgs/CompressedImage'
        })

        var image_topic_2 = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/RGB_image_viewer/matched/compressed',
            messageType: 'sensor_msgs/CompressedImage'
        })

        image_topic_1.subscribe(function (message) {
            document.getElementsByClassName("image-header-1").src = "data:image/png;base64," + message.data
            image_topic_1.unsubscribe();
        })

        image_topic_2.subscribe(function (message) {
            document.getElementsByClassName("image-header-2").src = "data:image/png;base64," + message.data
            image_topic_2.unsubscribe();
        })
    }

    render() {
        return (

            <button id="button_set" onClick={this.imageSubs}>Subscribe</button>

        )
    }
}

ReactDOM.render(e(SubscribeComponent), document.querySelector('#bttn_shell'))

