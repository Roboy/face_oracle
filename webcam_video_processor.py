#!/usr/bin/env python
import face_recognition
import cv2
# import asyncio
# import websockets
import pickle
import rospy
# from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import CompressedImage, PointCloud2
from roboy_cognition_msgs.srv import RecognizeFaces, RecognizeFacesRequest
from roboy_cognition_msgs.msg import FacialFeatures, RecognizedFaces, Faces
from geometry_msgs.msg import Point
from roboy_control_msgs.msg import Strings
import numpy as np
import websocket
import ssl
import time
import pdb
import signal
import sys
import argparse

import cv2
from PIL import Image
import threading
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from SocketServer import ThreadingMixIn
import StringIO
import time

from argparse import ArgumentParser

argdef = ArgumentParser(description="This script converts OpenCV camera input to Roboy ROS Faces messages.")
argdef.add_argument(
    "--input", "-i",
    dest="source",
    default=0,
    nargs=1,
    help="Input for OpenCV2 VideoCapture, such as video device index or rtmp URL.")
argdef.add_argument(
    "--query", "-q",
    dest="query_endpoint",
    default="ws://bot.roboy.org:8765",
    nargs=1,
    help="FaceOracle query server websocket endpoint.")
argdef.add_argument(
    "--address", "-a",
    dest="ui_address",
    default="localhost:8088",
    nargs=1,
    help="Network Interface on which camera HTTP stream is served.")
args = argdef.parse_args()


# import ros_numpy
def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def frame_callback(frame):
    frame = increase_brightness(frame, 20)
    face_locations = []
    face_encodings = []
    face_names = []
    face_confidences = []
    process_this_frame = True

    # Resize frame of video to 1/4 size for faster face recognition processing
    scale_fator = 0.5
    small_frame = cv2.resize(frame, (0, 0), fx=scale_fator, fy=scale_fator)

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB) # small_frame[:, :, ::-1]


    # Only process every other frame of video to save time
    if process_this_frame:
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        face_names = []
        req = RecognizeFacesRequest()
        encodings = [FacialFeatures(ff=encoding) for encoding in face_encodings]
        if len(face_encodings) > 0:
            pickled_encodings = pickle.dumps((face_encodings, bytes(), "abc"), protocol=2)
            try:
                ws = websocket.create_connection(args.query_endpoint)
                ws.send_binary(pickled_encodings)
                pickled_results = ws.recv()
                ws.close()
                face_names, face_confidences, face_node_ids = pickle.loads(pickled_results)
            except Exception, e:
                print('Error: '+ str(e))
            msg = Faces()
            msg.names = face_names
            msg.confidence = face_confidences
            msg.ids = face_node_ids
            msg.face_encodings = [FacialFeatures(ff=x) for x in face_encodings]
            names_pub.publish(msg)

    process_this_frame = not process_this_frame

    scaled_face_locations = []
    # Display the results
    for (top, right, bottom, left), name, confidence in zip(face_locations, face_names, face_confidences):
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        top *= int(1/scale_fator)
        right *= int(1/scale_fator)
        bottom *= int(1/scale_fator)
        left *= int(1/scale_fator)
        if point_cloud is not None:
            # point_cloud = ros_numpy.numpify(pcl)
            pos3D = point_cloud[top][right]
            print("x: %f, y: %f, z: %f"%(pos3D[0], pos3D[1], pos3D[2]))
        # scaled_face_locations.append((top, right, bottom, left))

        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        # Draw a label with a name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        try:
            cv2.putText(frame, "%s %i%% "%(name, confidence*100), (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
        except:
            pass
        point = Point()
        point.x = left+(right-left)/2.0-1280/2.0
        point.y = top+(bottom-top)/2.0-720/2.0
        face_position_publisher.publish(point)
        # print("x: %f, y: %f"%(left+(right-left)/2.0-1280/2.0,top+(bottom-top)/2.0-720/2.0))

    # Display the resulting image
    hsvImg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsvImg[...,1] = hsvImg[...,1]*0.9
    hsvImg[...,2] = hsvImg[...,2]*0.9
    global marked_frame
    marked_frame = cv2.cvtColor(hsvImg, cv2.COLOR_HSV2BGR)


class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global marked_frame
        if self.path.endswith('.mjpg'):
            try:
                print("sending")
                self.send_response(200)
                self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
                self.end_headers()
                while not rospy.is_shutdown():
                    try:
                        ret, frame = video_capture.read()
                        frame_callback(frame)
                        # rc,img = capture.read()
                        if marked_frame is None:
                            continue
                        imgRGB=cv2.cvtColor(marked_frame,cv2.COLOR_BGR2RGB)
                        jpg = Image.fromarray(imgRGB)
                        tmpFile = StringIO.StringIO()
                        jpg.save(tmpFile,'JPEG')
                        self.wfile.write("--jpgboundary")
                        self.send_header('Content-type','image/jpeg')
                        self.send_header('Content-length',str(tmpFile.len))
                        self.end_headers()
                        jpg.save(self.wfile,'JPEG')
                        time.sleep(0.01)
                    except KeyboardInterrupt:
                        break
            except:
                pass            
            return
        if self.path.endswith('.html'):
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>')
            self.wfile.write('<img src="http://'+args.ui_address+'/cam.mjpg"/>')
            self.wfile.write('</body></html>')
            return


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

global point_cloud, marked_frame, video_capture
point_cloud = None
marked_frame = None

rospy.init_node('face_encodings_extractor')

publish_names_srv = rospy.ServiceProxy('/roboy/cognition/vision/face_encodings', RecognizeFaces)
face_position_publisher = rospy.Publisher('roboy/cognition/vision/face_coordinates', Point, queue_size=1)
names_pub = rospy.Publisher('/roboy/cognition/vision/visible_face_names', Faces, queue_size=1)
video_capture = cv2.VideoCapture(args.source)

def signal_handler(sig, frame):
    # print('You pressed Ctrl+C!')
    video_capture.release
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

try:
    server = ThreadedHTTPServer(args.ui_address.split(":"), CamHandler)
    print "server started"
    server.serve_forever()
except KeyboardInterrupt:
    video_capture.release()
    server.socket.close()
    cv2.destroyAllWindows()
