import websocket
import ssl
import rospy
import pickle
import numpy as np

from ecdsa import VerifyingKey
import hashlib

from roboy_cognition_msgs.msg import FacialFeatures, RecognizedFaces

from roboy_cognition_msgs.srv import RecognizeFaces, RecognizeFacesResponse
from roboy_middleware_msgs.srv import OptigaSign, OptigaSignResponse

import pdb

def decode_DER_signature(dirty_signature):
    if dirty_signature[2] == '\x00':
        dirty_signature = dirty_signature[:2] + dirty_signature[3:]

    r = dirty_signature[2:34]

    if dirty_signature[36] == '\x00':
        dirty_signature = dirty_signature[:36] + dirty_signature[37:]

    s = dirty_signature[36:]

    return r + s

def cb(req):
    encodings = [np.array(e.ff) for e in req.encodings]

    # sign with optiga
    # pdb.set_trace()
    to_hash = bytes(encodings)#array.array('B', encodings).tostring()
    h = hashlib.sha256(to_hash).digest()
    res = optiga_srv(h)
    signature = decode_DER_signature(res.signature)

    # # # test verify
    # vk = VerifyingKey.from_pem(open("pubkey.pem").read())
    # vk.verify_digest(signature, h)
    # print("verified")

    pickled_encodings = pickle.dumps((encodings, h, signature), protocol=2)

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
optiga_srv = rospy.ServiceProxy('optiga', OptigaSign)
rospy.spin()
