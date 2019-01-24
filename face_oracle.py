import asyncio
import websockets
import pyroboy
import redis
import struct, pickle
from pyroboy.face_recognition import FaceRec
import numpy as np

class FaceOracle:
    def __init__(self):
        self.r = redis.Redis(host='172.17.0.1', port=6379, db=0)
        self.start_server = websockets.serve(self.recognize, 'localhost', 8765)

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
        face_encoding = pickle.loads(b_face_encoding)#.decode()
        #face_encoding = struct.unpack('%sd' % 128, b_face_encoding)
        ids, known_faces = self.get_known_faces()
        idx = FaceRec.match_face(face_encoding, known_faces)
        if idx:
            await websocket.send(pickle.dumps(int(ids[idx].decode('utf-8'))))
        await websocket.send(pickle.dumps(-1))

if __name__ == '__main__':
    oracle = FaceOracle()
