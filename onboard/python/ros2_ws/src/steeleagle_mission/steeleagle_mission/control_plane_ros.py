#!/usr/bin/env python3

# Copyright (C) 2024 Carnegie Mellon University
# SPDX-FileCopyrightText: 2024 Carnegie Mellon University - Satyalab
#
# SPDX-License-Identifier: GPL-2.0-only

import sys
import logging
from google.protobuf.message import DecodeError
from google.protobuf import text_format
from cnc_protocol import cnc_pb2
import argparse
import zmq
import redis
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

class ControlPlane(Node):

    def __init__(self, drones):
        super().__init__('control_plane')
        self.drones = drones
        self.publisher = self.create_publisher(String, 'manual_control', 10)
        self.get_logger().info("Control Plane Node initialized")

    def publish_request(self, drone_id, request):
        self.get_logger().info(f'Publishing request for drone: {drone_id}')
        msg = String()
        msg.data = request
        self.publisher.publish(msg)
        self.get_logger().info(f'Message published: {msg.data}')

def listen_cmdrs(sock, drones, redis, control_plane):
    while rclpy.ok():
        rclpy.spin_once(control_plane, timeout_sec=0)
        try:
            msg = sock.recv()  # Blocking call, waits for a message
            extras = cnc_pb2.Extras()
            extras.ParseFromString(msg)
            request = text_format.MessageToString(extras)
            logger.info(f'Request received:\n{request}')
            drones[extras.cmd.for_drone_id] = request
            sock.send(b'ACK')
            # key = redis.xadd(
            #     f"commands",
            #     {"commander": extras.commander_id, "drone": extras.cmd.for_drone_id, "value": text_format.MessageToString(extras),}
            # )
            # logger.debug(f"Updated redis under stream commands at key {key}")
            control_plane.publish_request(extras.cmd.for_drone_id, request)
        except DecodeError:
            sock.send(b'Error decoding protobuf. Did you send a cnc_pb2?')

        

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--droneport', type=int, default=6000, help='Specify port to listen for drone requests [default: 6000]')
    parser.add_argument('-c', '--cmdrport', type=int, default=6001, help='Specify port to listen for commander requests [default: 6001]')
    # parser.add_argument(
    #     "-r", "--redis", type=int, default=6379, help="Set port number for redis connection [default: 6379]"
    # )
    # parser.add_argument(
    #     "-a", "--auth", default="", help="Share key for redis user."
    # )
    args = parser.parse_args()

    # r = redis.Redis(host='redis', port=args.redis, username='steeleagle', password=f'{args.auth}', decode_responses=True)
    # logger.info(f"Connected to redis on port {args.redis}...")
    r = None
    drones = {}

    rclpy.init(args=None)
    control_plane = ControlPlane(drones)

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(f'tcp://*:{args.cmdrport}')

    logger.info(f'Listening on tcp://*:{args.cmdrport} for commander requests...')

    try:
        listen_cmdrs(sock, drones, r, control_plane)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        control_plane.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
