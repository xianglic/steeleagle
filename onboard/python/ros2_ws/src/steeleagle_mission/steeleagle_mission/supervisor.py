import asyncio
import importlib
import os
import subprocess
import sys
from zipfile import ZipFile

import requests
import zmq
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import argparse


class Supervisor(Node):

    def __init__(self, args):
        print("Initializing Supervisor Node...")
        # ros
        super().__init__('supervisor')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Add the directories to the system path
        current_dir = os.path.dirname(os.path.abspath(__file__))
        print(f"Curr dir: {current_dir}")
        implementation_dir = os.path.join(current_dir, 'implementation')
        interfaces_dir = os.path.join(current_dir, 'interfaces')

        if implementation_dir not in sys.path:
            sys.path.append(implementation_dir)
        if interfaces_dir not in sys.path:
            sys.path.append(interfaces_dir)
        
        # original
        # Import the files corresponding to the selected drone/cloudlet
        drone_import = f"implementation.drones.{args.drone}"
        print(f"Importing drone module: {drone_import}")
        cloudlet_import = f"implementation.cloudlets.{args.cloudlet}"
        print(f"Importing cloudlet module: {cloudlet_import}")
        
        try:
            Drone = importlib.import_module(drone_import)
        except Exception as e:
            self.get_logger().error(f'Could not import drone {args.drone}')
            rclpy.shutdown()
            raise e
        try:
            Cloudlet = importlib.import_module(cloudlet_import)
        except Exception as e:
            self.get_logger().error(f'Could not import cloudlet {args.cloudlet}')
            rclpy.shutdown()
            raise e
        try:
            self.cloudlet = getattr(Cloudlet, args.cloudlet)()
        except Exception as e:
            self.get_logger().error(f'Could not initialize {args.cloudlet}, name does not exist. Aborting.')
            rclpy.shutdown()
            raise e
        try:
            kwargs = {}
            if args.sim:
                kwargs['sim'] = True
            if args.lowdelay:
                kwargs['lowdelay'] = True
            self.drone = getattr(Drone, args.drone)(**kwargs)
        except Exception as e:
            self.get_logger().error(f'Could not initialize {args.drone}, name does not exist. Aborting.')
            rclpy.shutdown()

        # Set the Gabriel soure
        self.source = 'telemetry'
        self.reload = False
        self.mission = None
        self.missionTask = None
        self.manual = True # Default to manual control
        self.heartbeats = 0
        self.zmq = zmq.Context().socket(zmq.REQ)
        self.zmq.connect(f'tcp://{args.server}:{args.zmqport}')
        self.tlogfile = None
        if args.trajectory:
            self.tlogfile = open("trajectory.log", "w")
            
    # other methods unchanged...
    async def initializeConnection(self):
        await self.drone.connect()
        await self.drone.startStreaming()
        self.cloudlet.startStreaming(self.drone, 'coco', 30)
        
        
        
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


async def main(args=None):
    print("Starting main function...")

    #args
    parser = argparse.ArgumentParser(prog='supervisor',
        description='Bridges python API drones to SteelEagle.')
    parser.add_argument('-d', '--drone', default='ParrotAnafiDrone',
                        help='Set the type of drone to interface with [default: ParrotAnafiDrone]')
    parser.add_argument('-c', '--cloudlet', default='PureOffloadCloudlet',
                        help='Set the type of offload method to the cloudlet [default: PureOffloadCloudlet]')
    parser.add_argument('-s', '--server', default='gabriel-server',
                        help='Specify address of Steel Eagle CNC server [default: gabriel-server]')
    parser.add_argument('-p', '--port', default='9099',
                        help='Specify websocket port [default: 9099]')
    parser.add_argument('-l', '--loglevel', default='INFO',
                        help='Set the log level')
    parser.add_argument('-S', '--sim', action='store_true',
        help='Connect to simulated drone instead of a real drone [default: False]')
    parser.add_argument('-L', '--lowdelay', action='store_true',
        help='Use low delay settings for video streaming [default: False]')
    parser.add_argument('-zp', '--zmqport', type=int, default=6000,
                        help='Specify websocket port [default: 6000]')
    parser.add_argument('-t', '--trajectory', action='store_true',
        help='Log the trajectory of the drone over the flight duration [default: False]')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    
    print(f"Parsed arguments: {args}")
    
    # ros node
    print("Initializing ROS 2...")
    rclpy.init(args=rclpy.utilities.remove_ros_args(sys.argv))
    supervisor = Supervisor(args)
    
    await supervisor.initializeConnection()
    
    if rclpy.ok():
        print("Spinning supervisor node...")
        rclpy.spin(supervisor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    supervisor.destroy_node()
    rclpy.shutdown()
    print("ROS 2 shutdown complete.")


if __name__ == '__main__':
    asyncio.run(main())
