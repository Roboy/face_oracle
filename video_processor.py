import face_recognition
import cv2
# import asyncio
# import websockets
import pickle
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import CompressedImage, PointCloud2
from roboy_cognition_msgs.srv import RecognizeFaces, RecognizeFacesRequest
from roboy_cognition_msgs.msg import FacialFeatures, RecognizedFaces
from roboy_control_msgs.msg import Strings
import numpy as np
import websocket
import ssl
import time
import pdb
import ros_numpy

# Get a reference to webcam #0 (the default one)
# video_capture = cv2.VideoCapture(0)

def zed_pcl_callback(data):
    global point_cloud
    point_cloud = ros_numpy.numpify(data)

def zed_frame_callback(img):

    face_locations = []
    face_encodings = []
    face_names = []
    face_confidences = []
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
    # pdb.set_trace()
    if img.header.seq%3 == 0:
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        face_names = []
        req = RecognizeFacesRequest()
        req.encodings = [FacialFeatures(ff=encoding) for encoding in face_encodings]
        if len(face_encodings) > 0:
            resp = publish_names_srv(req)
            face_names = resp.names
            face_confidences = resp.confidence

    process_this_frame = not process_this_frame

    scaled_face_locations = []
    # Display the results
    for (top, right, bottom, left), name, confidence in zip(face_locations, face_names, face_confidences):
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4
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
        cv2.putText(frame, "%s %i%% "%(name, confidence*100), (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

    # Display the resulting image
    cv2.imshow('Video', frame)
    cv2.waitKey(1)
    # Hit 'q' on the keyboard to quit!
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break


global point_cloud
point_cloud = None

rospy.init_node('face_encodings_extractor')

# pcl_sub = message_filters.Subscriber('/zed/point_cloud/cloud_registered', PointCloud2)#, zed_pcl_callback)
# img_sub = message_filters.Subscriber('/zed/left/image_rect_color/compressed', CompressedImage)#, zed_frame_callback)
# ts = message_filters.ApproximateTimeSynchronizer([img_sub, pcl_sub], 10, 2, allow_headerless=True)
# ts.registerCallback(zed_frame_callback)
img_sub = rospy.Subscriber('/zed/left/image_rect_color/compressed', CompressedImage, zed_frame_callback)
# rospy.wait_for_service('/roboy/cognition/vision/face_encodings')
publish_names_srv = rospy.ServiceProxy('/roboy/cognition/vision/face_encodings', RecognizeFaces)
rospy.spin()

cv2.destroyAllWindows()
