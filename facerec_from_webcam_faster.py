import face_recognition
import cv2
# import asyncio
# import websockets
import pickle
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud2
from roboy_cognition_msgs.msg import FaceCoordinates
from roboy_control_msgs.msg import Strings
import numpy as np
import websocket
import ssl
import time
import pdb
import ros_numpy
# import message_filters
# This is a demo of running face recognition on live video from your webcam. It's a little more complicated than the
# other example, but it includes some basic performance tweaks to make things run a lot faster:
#   1. Process each video frame at 1/4 resolution (though still display it at full resolution)
#   2. Only detect faces in every other frame of video.

# PLEASE NOTE: This example requires OpenCV (the `cv2` library) to be installed only to read from your webcam.
# OpenCV is *not* required to use the face_recognition library. It's only required if you want to run this
# specific demo. If you have trouble installing it, try any of the other demos that don't require it instead.

# Get a reference to webcam #0 (the default one)
# video_capture = cv2.VideoCapture(0)

# images = [
#     "alona.jpg",
#     "sausy.png",
#     "simon.jpg",
#     "wagram.jpg"
# ]
#
# path = "/home/missxa/tmp/known_faces/"

# Create arrays of known face encodings and their names
# known_face_encodings = []
# known_face_names = []
#
# for im in images:
#     loaded_image = face_recognition.load_image_file(path+im)
#     known_face_encodings.append(face_recognition.face_encodings(loaded_image)[0])
#     known_face_names.append(im.split(".")[0])

def zed_pcl_callback(data):
    global point_cloud
    point_cloud = ros_numpy.numpify(data)

def zed_frame_callback(img):
    face_locations = []
    face_encodings = []
    face_names = []
    process_this_frame = True
    # Grab a single frame of video
    # ret, frame = video_capture.read()
    np_arr = np.fromstring(img.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Resize frame of video to 1/4 size for faster face recognition processing
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_small_frame = small_frame[:, :, ::-1]


    # Only process every other frame of video to save time
    if process_this_frame:
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        face_names = []
        if len(face_encodings) > 0:
            ws = websocket.create_connection("wss://localhost:8765", sslopt=sslopt)
            pickled_encodings = pickle.dumps(face_encodings[0], protocol=2)
            ws.send_binary(pickled_encodings)
            # import pdb; pdb.set_trace()
            pickled_names = ws.recv()
            face_names = pickle.loads(pickled_names)
            ws.close()
            names_pub.publish(Strings(face_names))

        # for face_encoding in face_encodings:
        #     # See if the face is a match for the known face(s)
        #
        #     matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        #     name = "unknown"
        #
        #     # If a match was found in known_face_encodings, just use the first one.
        #     if True in matches:
        #         first_match_index = matches.index(True)
        #         name = known_face_names[first_match_index]
        #
        #     face_names.append(name)

    process_this_frame = not process_this_frame

    # pdb.set_trace()
    scaled_face_locations = []
    # Display the results
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4
        # pixelTo3DPoint(top, right)
        if point_cloud is not None:
            # point_cloud = ros_numpy.numpify(pcl)
            pos3D = point_cloud[top][right]
            print("x: %f, y: %f, z: %f"%(pos3D[0], pos3D[1], pos3D[2]))
        scaled_face_locations.append((top, right, bottom, left))
        # pdb.set_trace()

        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        # Draw a label with a name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

    # Display the resulting image
    cv2.imshow('Video', frame)
    cv2.waitKey(1)
    # Hit 'q' on the keyboard to quit!
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

sslopt = dict(cert_reqs=ssl.CERT_REQUIRED)
sslopt['ca_certs']='test_localhost.pem'
global point_cloud
point_cloud = None

rospy.init_node('face_oracle_client')

# pcl_sub = message_filters.Subscriber('/zed/point_cloud/cloud_registered', PointCloud2)#, zed_pcl_callback)
# img_sub = message_filters.Subscriber('/zed/left/image_rect_color/compressed', CompressedImage)#, zed_frame_callback)
# ts = message_filters.ApproximateTimeSynchronizer([img_sub, pcl_sub], 10, 2, allow_headerless=True)
# ts.registerCallback(zed_frame_callback)
img_sub = rospy.Subscriber('/zed/left/image_rect_color/compressed', CompressedImage, zed_frame_callback)
names_pub = rospy.Publisher('/roboy/cognition/visible_face_names', Strings, queue_size=1)
rospy.spin()

cv2.destroyAllWindows()
