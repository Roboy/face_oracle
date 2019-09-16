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
from argparse import ArgumentParser

from ecdsa import VerifyingKey
import hashlib

argdef = ArgumentParser(description="This script hosts a websocker server that processes face recognition requests.")
argdef.add_argument(
    "--redis-pass",
    dest="redis_pass",
    default=os.environ['REDIS_PASSWORD'] if 'REDIS_PASSWORD' in os.environ else "",
    help="Redis password")
argdef.add_argument(
    "--redis-port",
    dest="redis_port",
    default=6379,
    help="Redis port.")
argdef.add_argument(
    "--redis-host",
    dest="redis_host",
    default="bot.roboy.org",
    help="Redis host.")
args = argdef.parse_args()

print(f"[face_oracle server]: Startup")
print(f"[face_oracle server]: Using redis host {args.redis_host} at port {args.redis_port}")
print(f"[face_oracle server]: Using neo4j at {os.environ['NEO4J_ADDRESS']}")

class FaceOracle:
    def __init__(self):
        self.r = redis.Redis(
            host=args.redis_host,
            port=args.redis_port,
            password=args.redis_pass,
            db=0)
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
        print(f"[face_oracle server]: Received query.")
        #print(b_face_encoding)
        face_encodings, h, signature = pickle.loads(b_face_encodings, encoding='bytes')#.decode()
        print(f"[face_oracle server]: Unpacked query.") 

        # check signature
        # to_hash = bytearray(face_encodings)#array.array('B', encodings).tostring()
        # h = hashlib.sha256(to_hash).digest()
        # vk = VerifyingKey.from_pem(open("newpubkey.pem").read())
        # try: 
        #     vk.verify_digest(signature, h)
        # except:
        #     print("Some dirty business here. Could not verify signature")

        #face_encoding = struct.unpack('%sd' % 128, b_face_encoding)
        ids, known_faces = self.get_known_faces()
        print(f"[face_oracle server]: Matching {len(face_encodings)} faces among {len(ids)} known faces.")
        names = []
        confidences = []
        node_ids = []
        for face_encoding in face_encodings:
            try:
                if len(known_faces) > 0:
                    print(f"[face_oracle server]: Calling FaceRec.match_face(...)")
                    idx, confidence = FaceRec.match_face(face_encoding, known_faces)
                    if idx is not None:
                        node = sess.retrieve(node_id=int(ids[idx].decode('utf-8')))[0]
                        node_ids.append(int(ids[idx].decode('utf-8')))
                        names.append(node.get_name())
                        confidences.append(confidence)
                        continue
            except Exception as e:
                print(f"[face_oracle server]: ERROR: {e}")
            print(f"[face_oracle server]: Falling back to stranger id.")
            names.append("stranger")
            confidences.append(.0)
            node_ids.append(-1)
        print(f"[face_oracle server]: Building response")
        resp = (names, confidences, node_ids)
        print(f"[face_oracle server]: Sending response {resp}")
        await websocket.send(pickle.dumps(resp, protocol=2))
        print(f"[face_oracle server]: Sent!")

if __name__ == '__main__':
    global sess
    onto = Ontology(path_to_yaml="ravestate_ontology.yml")
    sess = Session(
        ontology=onto,
        neo4j_address=os.environ['NEO4J_ADDRESS'],
        neo4j_username=os.environ['NEO4J_USERNAME'],
        neo4j_password=os.environ['NEO4J_PASSWORD'])
    oracle = FaceOracle()
    oracle.start()
