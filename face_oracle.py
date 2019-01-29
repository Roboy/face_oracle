import asyncio
import websockets
import pyroboy
import redis
import struct, pickle
from pyroboy.face_recognition import FaceRec
import numpy as np
import pdb

from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node

class FaceOracle:
    def __init__(self):
        self.r = redis.Redis(host='172.17.0.1', port=6379, db=0)
        self.start_server = websockets.serve(self.recognize, '127.0.0.1', 8765)

    def start(self):
        asyncio.get_event_loop().run_until_complete(self.start_server)
        asyncio.get_event_loop().run_forever()

    def get_known_faces(self):
        # key = neo4j node id
        # value = binary face encoding

        keys = self.r.keys()
        values = []
        for k in keys:
            value = self.r.get(k)
            #import pdb; pdb.set_trace()
            values.append(pickle.loads(value))
        return keys,values

    async def recognize(self, websocket, path):
        b_face_encoding = await websocket.recv()
        #print(b_face_encoding)
        face_encoding = pickle.loads(b_face_encodings, encoding='bytes')#.decode()
        #face_encoding = struct.unpack('%sd' % 128, b_face_encoding)
        ids, known_faces = self.get_known_faces()
        names = []
        for face_encoding in face_encodings:
            idx = FaceRec.match_face(face_encoding, known_faces)
            if idx is not None:
                node = sess.retrieve(node_id=int(ids[idx].decode('utf-8')))[0]
                names.append(node.get_name())
            else:
                names.append("stranger")
        await websocket.send(pickle.dumps(names, protocol=2))        

if __name__ == '__main__':
    global sess
    onto = Ontology(path_to_yaml="ravestate_ontology.yml")
    sess = Session(
        ontology=onto,
        neo4j_address="",
        neo4j_username="",
        neo4j_password="")
    #res = sess.retrieve(node_id=7)
    #pdb.set_trace()
    oracle = FaceOracle()
    oracle.start()
