import os
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

from ecdsa import VerifyingKey
import hashlib

class FaceOracle:
    def __init__(self):
        self.r = redis.Redis(host='bot.roboy.org', port=6379, password=os.environ['REDIS_PASSWORD'], db=0)
        self.start_server = websockets.serve(self.recognize, '0.0.0.0', 8765)

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
        b_face_encodings = await websocket.recv()
        #print(b_face_encoding)
        face_encodings, h, signature = pickle.loads(b_face_encodings, encoding='bytes')#.decode()
        
        # check signature
        #to_hash = bytearray(face_encodings)#array.array('B', encodings).tostring()
        #h = hashlib.sha256(to_hash).digest()
        vk = VerifyingKey.from_pem(open("newpubkey.pem").read())
        #try: 
        #    vk.verify_digest(signature, h)
        #except:
        #    print("Some dirty business here. Could not verify signature")

        #face_encoding = struct.unpack('%sd' % 128, b_face_encoding)
        ids, known_faces = self.get_known_faces()
        names = []
        confidences = []
        node_ids = []
        for face_encoding in face_encodings:
            idx, confidence = FaceRec.match_face(face_encoding, known_faces)
            if idx is not None:
                node = sess.retrieve(node_id=int(ids[idx].decode('utf-8')))[0]
                node_ids.append(int(ids[idx].decode('utf-8')))
                names.append(node.get_name())
                confidences.append(confidence)
            else:
                names.append("stranger")
                confidences.append(1.0)
                node_ids.append(-1)
        await websocket.send(pickle.dumps((names, confidences, node_ids), protocol=2))

if __name__ == '__main__':
    global sess
    onto = Ontology(path_to_yaml="ravestate_ontology.yml")
    sess = Session(
        ontology=onto,
        neo4j_address=os.environ['NEO4J_ADDRESS'],
        neo4j_username=os.environ['NEO4J_USERNAME'],
        neo4j_password=os.environ['NEO4J_PASSWORD'])
    #res = sess.retrieve(node_id=7)
    #pdb.set_trace()
    oracle = FaceOracle()
    oracle.start()
