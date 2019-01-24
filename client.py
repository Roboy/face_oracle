import asyncio
import websockets
import pickle
import random
import numpy as np

from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node


async def hello():
    async with websockets.connect(
            'ws://127.0.0.1:8765') as websocket:
        encoding = np.zeros(128)#np.random.rand(128)
        encoding[0]=10
        await websocket.send(pickle.dumps(encoding))#.encode())#struct.pack('%sd' % len(encoding), *encoding))
        name = await websocket.recv()
        print("< name: {}".format(name))


asyncio.get_event_loop().run_until_complete(hello())
