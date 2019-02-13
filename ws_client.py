import websocket
import ssl
import rospy
import pickle
import numpy as np

from roboy_cognition_msgs.msg import FacialFeatures, RecognizedFaces
from roboy_cognition_msgs.srv import RecognizeFaces, RecognizeFacesResponse

def cb(req):
    encodings = [np.array(e.ff) for e in req.encodings]
    pickled_encodings = pickle.dumps(encodings, protocol=2)

    # ws = websocket.create_connection("wss://bot.roboy.org:8765", sslopt=sslopt)
    ws = websocket.create_connection("ws://bot.roboy.org:8765")
    ws.send_binary(pickled_encodings)
    pickled_results = ws.recv()
    ws.close()

    names, confidences = pickle.loads(pickled_results)

    msg = RecognizedFaces()
    msg.names = names
    msg.confidence = confidences
    names_pub.publish(msg)

    res = RecognizeFacesResponse()
    res.names = names
    res.confidence = confidences
    return res

sslopt = dict(cert_reqs=ssl.CERT_REQUIRED)
sslopt['ca_certs']='test_localhost.pem'
sslopt["check_hostname"] = False

rospy.init_node('face_oracle_client')
names_pub = rospy.Publisher('/roboy/cognition/vision/visible_face_names', RecognizedFaces, queue_size=1)
rospy.Service('/roboy/cognition/vision/face_encodings', RecognizeFaces, cb)
rospy.spin()
