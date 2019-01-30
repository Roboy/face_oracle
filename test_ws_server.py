#!/usr/bin/env python

# WS server example

import asyncio
import websockets
import ssl
import pathlib
import pickle
import pdb
import numpy as np

async def hello(websocket, path):
    packed_encodings = await websocket.recv()
    # pdb.set_trace()
    encodings = pickle.loads(packed_encodings, encoding='bytes')

    names = ['stranger']*len(encodings)
    await websocket.send(pickle.dumps(names, protocol=2))

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
ssl_context.load_cert_chain('test_localhost.pem')

start_server = websockets.serve(hello, '0.0.0.0', 8765, ssl=ssl_context)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
