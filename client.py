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
            'ws://localhost:8765') as websocket:
        encoding = np.zeros(128)#np.random.rand(128)
        await websocket.send(pickle.dumps(encoding))#.encode())#struct.pack('%sd' % len(encoding), *encoding))
        idx = await websocket.recv()
        print("< id: {}".format(pickle.loads(idx)))

sess = Session(
    ontology=onto,
    neo4j_address="bolt://localhost:7687",
    neo4j_username="neo4j",
    neo4j_password="test")


asyncio.get_event_loop().run_until_complete(hello())
