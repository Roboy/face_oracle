import rclpy

from time import sleep
from roboy_cognition_msgs.msg import FacialFeatures, RecognizedFaces


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_publisher')

    names_pub = node.create_publisher('/roboy/cognition/vision/visible_face_names', RecognizedFaces)



    msg = RecognizedFaces()

    i = 0
    while rclpy.ok():
        msg.names = ["wagram"]
        msg.confidence = np.zeros(len(msg.names))
        publisher.publish(msg)
        sleep(2)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
