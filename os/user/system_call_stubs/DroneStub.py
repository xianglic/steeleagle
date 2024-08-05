# SPDX-FileCopyrightText: 2023 Carnegie Mellon University - Satyalab
#
# SPDX-License-Identifier: GPL-2.0-only
import asyncio
import zmq

from cnc_protocol import cnc_pb2




class DroneStub():

    def __init__(self):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect("tcp://localhost:5555")
    
    ''' Streaming methods '''
    async def getCameras(self):
        driver_req = cnc_pb2.Driver()
        driver_req.getCameras.SetInParent()
        serialized_req = driver_req.SerializedToString()
        await self.socket.send(serialized_req)
        response = await self.socket.recv()
        print("Received response:", response)
        
        return response

    
    async def switchCameras(self):
        driver_req = cnc_pb2.Driver()
        driver_req.switchCameras.SetInParent()
        serialized_req = driver_req.SerializedToString()
        await self.socket.send(serialized_req)
        response = await self.socket.recv()
        print("Received response:", response)
        
        return response
       
    
    ''' Movement methods '''

    async def setAttitude(self):
        driver_req = cnc_pb2.Driver()
        driver_req.setAttitude.SetInParent()
        serialized_req = driver_req.SerializedToString()
        await self.socket.send(serialized_req)
        response = await self.socket.recv()
        print("Received response:", response)
        
        return response
       
    
    async def setVelocity(self):
        driver_req = cnc_pb2.Driver()
        driver_req.setVelocity.SetInParent()
        serialized_req = driver_req.SerializedToString()
        await self.socket.send(serialized_req)
        response = await self.socket.recv()
        print("Received response:", response)
        
        return response
    
    async def setRelativePosition(self):
        driver_req = cnc_pb2.Driver()
        driver_req.setRelativePosition.SetInParent()
        serialized_req = driver_req.SerializedToString()
        await self.socket.send(serialized_req)
        response = await self.socket.recv()
        print("Received response:", response)
        
        return response
    
    async def setTranslation(self):
        driver_req = cnc_pb2.Driver()
        driver_req.setTranslation.SetInParent()
        serialized_req = driver_req.SerializedToString()
        await self.socket.send(serialized_req)
        response = await self.socket.recv()
        print("Received response:", response)
        
        return response
    
    async def setGlobalPosition(self):
        driver_req = cnc_pb2.Driver()
        driver_req.setGlobalPosition.SetInParent()
        serialized_req = driver_req.SerializedToString()
        await self.socket.send(serialized_req)
        response = await self.socket.recv()
        print("Received response:", response)
        
        return response
    
    async def hover(self):
        driver_req = cnc_pb2.Driver()
        driver_req.hover.SetInParent()
        serialized_req = driver_req.SerializedToString()
        await self.socket.send(serialized_req)
        response = await self.socket.recv()
        print("Received response:", response)
        
        return response