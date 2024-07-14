import asyncio
import nest_asyncio
nest_asyncio.apply()
import importlib
import os
import subprocess
import sys
import time
from zipfile import ZipFile
from syncer import sync
import requests
import validators
import zmq
from cnc_protocol import cnc_pb2
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import argparse
from gabriel_protocol import gabriel_pb2
from gabriel_client.websocket_client import ProducerWrapper, WebsocketClient


class Supervisor(Node):

    def __init__(self, args):
        # ros
        super().__init__('supervisor')
        self.get_logger().info("Initializing Supervisor Node...")
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        
        # Import the files corresponding to the selected drone/cloudlet
        drone_import = f"implementation.drones.{args.drone}"
        self.get_logger().info(f"Importing drone module: {drone_import}")
        cloudlet_import = f"implementation.cloudlets.{args.cloudlet}"
        self.get_logger().info(f"Importing cloudlet module: {cloudlet_import}")
        
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

        # Set the Gabriel source
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
            
    async def initializeConnection(self):
        await self.drone.connect()
        await self.drone.startStreaming()
        self.cloudlet.startStreaming(self.drone, 'coco', 30)
    async def executeFlightScript(self, url: str):
        self.get_logger().info('Starting flight plan download...')
        try:
            self.download(url)
        except Exception as e:
            self.get_logger().info('Flight script download failed! Aborting.')
            return
        self.get_logger().info('Flight script downloaded...')
        self.start_mission()

    def start_mission(self):
        self.get_logger().info('Start mission supervisor')
        self.get_logger().info(self)
        # Stop existing mission (if there is one)
        self.stop_mission()
        # Start new task
        self.get_logger().info('MS import')
        if not self.reload:
            import mission
            import task_defs
            import transition_defs
        else:
            self.get_logger.info('Reloading...')
            modules = sys.modules.copy()
            for module in modules.values():
                if module.__name__.startswith('mission') or module.__name__.startswith('task_defs') or module.__name__.startswith('transition_defs'):
                    importlib.reload(module)
        self.get_logger().info('MC init')
        from mission.MissionController import MissionController
        self.mission = MissionController(self.drone, self.cloudlet)
        self.get_logger().info('Running flight script!')
        self.missionTask = asyncio.create_task(self.mission.run())
        self.reload = True

    def stop_mission(self):
        if self.mission and not self.missionTask.cancelled():
            self.get_logger().info('Mission script stop signalled')
            self.missionTask.cancel()
            self.mission = None
            self.missionTask = None

    def download(self, url: str):
        #download zipfile and extract reqs/flight script from cloudlet
        try:
            filename = url.rsplit(sep='/')[-1]
            self.get_logger().info(f'Writing {filename} to disk...')
            r = requests.get(url, stream=True)
            with open(filename, mode='wb') as f:
                for chunk in r.iter_content():
                    f.write(chunk)
            z = ZipFile(filename)
            try:
                subprocess.check_call(['rm', '-rf', './task_defs', './mission', './transition_defs'])
            except subprocess.CalledProcessError as e:
                self.get_logger().info(f"Error removing old task/transition defs: {e}")
            z.extractall()
            self.install_prereqs()
        except Exception as e:
            print(e)

    def install_prereqs(self) -> bool:
        ret = False
        # Pip install prerequsites for flight script
        try:
            subprocess.check_call(['python3', '-m', 'pip', 'install', '-r', './requirements.txt'])
            ret = True
        except subprocess.CalledProcessError as e:
            self.get_logger().info(f"Error pip installing requirements.txt: {e}")
        return ret


    async def commandHandler(self):
        name = await self.drone.getName()

        req = cnc_pb2.Extras()
        req.drone_id = name
        while True:
            await asyncio.sleep(0)
            try:
                self.zmq.send(req.SerializeToString())
                rep = self.zmq.recv()
                if b'No commands.' != rep:
                    extras  = cnc_pb2.Extras()
                    extras.ParseFromString(rep)
                    if extras.cmd.rth:
                        self.get_logger().info('RTH signaled from commander')
                        self.stop_mission()
                        self.manual = False
                        asyncio.create_task(self.drone.rth())
                    elif extras.cmd.halt:
                        self.get_logger().info('Killswitch signaled from commander')
                        self.stop_mission()
                        self.manual = True
                        self.get_logger().info('Manual control is now active!')
                        # Try cancelling the RTH task if it exists
                        sync(self.drone.hover())
                    elif extras.cmd.script_url:
                        # Validate url
                        if validators.url(extras.cmd.script_url):
                            self.get_logger().info(f'Flight script sent by commander: {extras.cmd.script_url}')
                            self.manual = False
                            asyncio.create_task(self.executeFlightScript(extras.cmd.script_url))
                        else:
                            self.get_logger().info(f'Invalid script URL sent by commander: {extras.cmd.script_url}')
                    elif self.manual:
                        if extras.cmd.takeoff:
                            self.get_logger().info(f'Received manual takeoff')
                            asyncio.create_task(self.drone.takeOff())
                        elif extras.cmd.land:
                            self.get_logger().info(f'Received manual land')
                            asyncio.create_task(self.drone.land())
                        else:
                            self.get_logger().info(f'Received manual PCMD')
                            pitch = extras.cmd.pcmd.pitch
                            yaw = extras.cmd.pcmd.yaw
                            roll = extras.cmd.pcmd.roll
                            gaz = extras.cmd.pcmd.gaz
                            gimbal_pitch = extras.cmd.pcmd.gimbal_pitch
                            self.get_logger().info(f'Got PCMD values: {pitch} {yaw} {roll} {gaz} {gimbal_pitch}')
                            asyncio.create_task(self.drone.PCMD(roll, pitch, yaw, gaz))
                            current = await self.drone.getGimbalPitch()
                            asyncio.create_task(self.drone.setGimbalPose(0, current+gimbal_pitch , 0))
                if self.tlogfile: # Log trajectory IMU data
                    speeds = await self.drone.getSpeedRel()
                    fspeed = speeds["speedX"]
                    hspeed = speeds["speedY"]
                    self.tlogfile.write(f"{time.time()},{fspeed},{hspeed} ")
            except Exception as e:
                self.get_logger().info(e)


    '''
    Process results from engines.
    Forward openscout engine results to Cloudlet object
    Parse and deal with results from command engine
    '''
    def processResults(self, result_wrapper):
        if self.cloudlet and result_wrapper.result_producer_name.value != 'telemetry':
            #forward result to cloudlet
            self.cloudlet.processResults(result_wrapper)
            return
        else:
            #process result from command engine
            pass

    def get_producer_wrappers(self):
        async def producer():
            await asyncio.sleep(0)
            self.heartbeats += 1
            input_frame = gabriel_pb2.InputFrame()
            input_frame.payload_type = gabriel_pb2.PayloadType.TEXT
            input_frame.payloads.append('heartbeart'.encode('utf8'))

            extras = cnc_pb2.Extras()
            try:
                extras.drone_id = sync(self.drone.getName())
                extras.location.latitude = sync(self.drone.getLat())
                extras.location.longitude = sync(self.drone.getLng())
                extras.location.altitude = sync(self.drone.getRelAlt())
                self.get_logger().info(f'Latitude: {extras.location.latitude} Longitude: {extras.location.longitude} Altitude: {extras.location.altitude}')
                extras.status.battery = sync(self.drone.getBatteryPercentage())
                extras.status.rssi = sync(self.drone.getRSSI())
                extras.status.mag = sync(self.drone.getMagnetometerReading())
                extras.status.bearing = sync(self.drone.getHeading())
                self.get_logger().info(f'Battery: {extras.status.battery} RSSI: {extras.status.rssi}  Magnetometer: {extras.status.mag} Heading: {extras.status.bearing}')
            except Exception as e:
                self.get_logger().info(f'Error getting telemetry: {e}')

            # Register on the first frame
            if self.heartbeats == 1:
                extras.registering = True

            self.get_logger().info('Producing Gabriel frame!')
            input_frame.extras.Pack(extras)
            return input_frame

        return ProducerWrapper(producer=producer, source_name=self.source)

        
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


async def async_main(args=None):
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
    asyncio.create_task(supervisor.commandHandler())

    print("Launching Gabriel")
    gabriel_client = WebsocketClient(
        args.server, args.port,
        [supervisor.get_producer_wrappers(), supervisor.cloudlet.sendFrame()],  supervisor.processResults
    )
    
    gabriel_task = asyncio.create_task(gabriel_client.launch())
    try:
        await gabriel_task
    except KeyboardInterrupt:
        supervisor.cloudlet.stopStreaming()
        await supervisor.drone.stopStreaming()
        supervisor.destroy_node()
        rclpy.shutdown()
    
    if rclpy.ok():
        print("Spinning supervisor node...")
        rclpy.spin(supervisor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    supervisor.destroy_node()
    rclpy.shutdown()
    print("ROS 2 shutdown complete.")
    

def main():
    asyncio.run(async_main())

if __name__ == '__main__':
    main()
