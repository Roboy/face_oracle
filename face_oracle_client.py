import websocket
import ssl
from roboy_cognition_msgs.msg import FacialFeatures, RecognizedFaces
from roboy_cognition_msgs.srv import RecognizeFaces

def cb(req):
    pickled_encodings = pickle.dumps(req.encodings, protocol=2)

    ws = websocket.create_connection("wss://bot.roboy.org:8765", sslopt=sslopt)
    ws.send_binary(pickled_encodings)
    pickled_names = ws.recv()
    ws.close()

    face_names = pickle.loads(pickled_names)
    # TODO extract confidence from face recognition
    confidence = np.random.randint(99, size=len(face_names))/100.0

    msg = RecognizedFaces()
    msg.names = face_names
    msg.confidence = confidence
    names_pub.publish(msg)

    res = RecognizeFacesResponse()
    res.names = face_names
    res.confidence = confidence
    return res

sslopt = dict(cert_reqs=ssl.CERT_REQUIRED)
sslopt['ca_certs']='test_localhost.pem'
sslopt["check_hostname"] = False

rospy.init_node('face_oracle_client')
names_pub = rospy.Publisher('/roboy/cognition/vision/visible_face_names', RecognizedFaces, queue_size=1)
rospy.Service('/roboy/cognition/vision/face_encodings', RecognizeFaces, cb)
