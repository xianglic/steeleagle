import asyncio
import os
import nest_asyncio
nest_asyncio.apply()
import importlib
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

import importlib.util

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
        self.manual = True # Default to manual control
        self.heartbeats = 0
        self.zmq = zmq.Context().socket(zmq.REQ)
        self.zmq.connect(f'tcp://{args.server}:{args.zmqport}')
        self.tlogfile = None
        if args.trajectory:
            self.tlogfile = open("trajectory.log", "w")
            
    '''
    ROS CONTROL
    '''        
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
                
    '''
    DRONE CONTROL
    '''
    async def initializeConnection(self):
        await self.drone.connect()
        await self.drone.startStreaming()
        self.cloudlet.startStreaming(self.drone, 'coco', 30)
        

    async def commandHandler(self):
        name = await self.drone.getName()

        req = cnc_pb2.Extras()
        req.drone_id = name
        while True:
            await asyncio.sleep(0)
            # print("COMMAND")
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

    def sendTelemetry(self):
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
                self.get_logger().debug(f'Latitude: {extras.location.latitude} Longitude: {extras.location.longitude} Altitude: {extras.location.altitude}')
                extras.status.battery = sync(self.drone.getBatteryPercentage())
                extras.status.rssi = sync(self.drone.getRSSI())
                extras.status.mag = sync(self.drone.getMagnetometerReading())
                extras.status.bearing = sync(self.drone.getHeading())
                self.get_logger().debug(f'Battery: {extras.status.battery} RSSI: {extras.status.rssi}  Magnetometer: {extras.status.mag} Heading: {extras.status.bearing}')
            except Exception as e:
                self.get_logger().debug(f'Error getting telemetry: {e}')

            # Register on the first frame
            if self.heartbeats == 1:
                extras.registering = True

            self.get_logger().debug('Producing Gabriel frame!')
            input_frame.extras.Pack(extras)
            return input_frame

        return ProducerWrapper(producer=producer, source_name=self.source)
    
    '''
    STREAM CONTROL
    '''
    def processResults(self, result_wrapper):
        if self.cloudlet and result_wrapper.result_producer_name.value != 'telemetry':
            #forward result to cloudlet
            self.cloudlet.processResults(result_wrapper)
            return
        else:
            #process result from telemetry engine
            pass

    
    '''
    MISSION CONTROL
    '''
    async def executeFlightScript(self, url: str):
        self.get_logger().info('Flight script download starting...')
        try:
            colcon_prefix_path = os.getenv('COLCON_PREFIX_PATH')
            ros_ws_directory = os.path.abspath(os.path.join(colcon_prefix_path, '..'))
            download_path = os.path.join(ros_ws_directory, 'download')
            os.makedirs(download_path, exist_ok=True)
            self.download(url, download_path)
            self.get_logger().info('Flight script downloaded...')
            self.add_directory_to_sys_path(os.path.join(download_path, 'flight_plan'))
            self.get_logger().info('Flight script path added...')
        except Exception as e:
            self.get_logger().info('Flight script download failed! Aborting.')
            return
        
        self.start_mission()
        
    def start_mission(self):
        self.get_logger().info('Flight script starting...')
        # Stop existing mission (if there is one)
        self.stop_mission()
        # Start new task
        self.get_logger().info('Flight script importing...')
        if not self.reload:
            import mission
            import task_defs
            import transition_defs
            
        else:
            self.get_logger().info('Flight script reloading...')
            modules = sys.modules.copy()
            for module in modules.values():
                if module.__name__.startswith('mission') or module.__name__.startswith('task_defs') or module.__name__.startswith('transition_defs'):
                    importlib.reload(module)
        self.get_logger().info('Flight script initializing...')
        from mission.MissionController import MissionController
        self.mission = MissionController(self.drone, self.cloudlet)
        self.get_logger().info('Flight script running!')
        self.mission.start()
        self.reload = True

    def stop_mission(self):
        if self.mission and self.mission.is_alive():
            self.get_logger().info('Flight script stopping...')
            self.mission.stop()
            self.mission.join()
            self.mission = None
            self.get_logger().info('Flight script stopped!')
    
    def add_directory_to_sys_path(self, directory):
        try:
            if directory not in sys.path:
                sys.path.append(directory)
        except Exception as e:
            print(e)

    def download(self, url: str, download_path):
        #download zipfile and extract reqs/flight script from cloudlet
        try:
            filename = url.rsplit(sep='/')[-1]
            save_path = os.path.join(download_path, filename)
            self.get_logger().info(f'Writing {filename} to {save_path}...')
            
            r = requests.get(url, stream=True)
            with open(save_path, mode='wb') as f:
                for chunk in r.iter_content(chunk_size=8192):
                    f.write(chunk)
            
            z = ZipFile(save_path)
            try:
                flight_plan_path = os.path.join(download_path, 'flight_plan')
                subprocess.check_call(['rm', '-rf', flight_plan_path])
            except subprocess.CalledProcessError as e:
                self.get_logger().info(f"Error removing old task/transition defs: {e}")
            z.extractall(download_path)
            prereq_path = os.path.join(download_path, 'flight_plan')
            self.install_prereqs(prereq_path)
            
        except Exception as e:
            print(e)

    def install_prereqs(self, prereq_path) -> bool:
        ret = False
        # Pip install prerequsites for flight script
        self.get_logger().info('Installing prerequisites...')
        try:
            subprocess.check_call(['python3', '-m', 'pip', 'install', '-r', 'requirements.txt'], cwd=prereq_path)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Error installing prerequisites: {e}')
        return ret



'''
MAIN LOOP
'''
async def async_spin(supervisor):
    print("Spinning supervisor node...")
    while rclpy.ok():
        # print ("Spinning once")
        rclpy.spin_once(supervisor, timeout_sec = 0)
        await asyncio.sleep(0)

    
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
    
    # connect
    await supervisor.initializeConnection()
    
    # command handler task
    commander_task = asyncio.create_task(supervisor.commandHandler())
    
    # the async task
    spin_task = asyncio.create_task(async_spin(supervisor))
    

    print("Launching Gabriel")
    gabriel_client = WebsocketClient(
        args.server, args.port,
        [supervisor.sendTelemetry(), supervisor.cloudlet.sendFrame()],  supervisor.processResults
    )
   
    try:
        gabriel_client.launch()
    except KeyboardInterrupt:
        # shut down supvisor
        supervisor.cloudlet.stopStreaming()
        await supervisor.drone.stopStreaming()
        commander_task.cancel()
        
        # shut down ros
        spin_task.cancel()
        supervisor.destroy_node()
        rclpy.shutdown()
        

def main():
    asyncio.run(async_main())

if __name__ == '__main__':
    main()
