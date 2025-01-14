#!/usr/bin/python3
"""slamtec.py
This module provides a class to interface with the Slamtec Mapper device. It allows for connecting to the device, 
sending commands, receiving responses, and handling various functionalities such as getting map data, laser scan data, 
localization status, and more.

Classes:
    SlamtecMapper: A class to manage the connection and communication with the Slamtec Mapper device.
Functions:
    show_summary(st): Show a summary of the map and localization information.
    show_map(map_data): Display the map data as an image.
"""
import socket
import json
import base64
import math
from pathlib import Path
import struct
import time
import sys


class SlamtecMapper:
    """SlamtecMapper class for interfacing with a Slamtec Mapper device.
    This class provides methods to connect to the Slamtec Mapper, send commands, and retrieve data such as the known area, pose, map data, laser scan data, and various configurations.
    Attributes:
        socket (socket.socket): The socket object for communication.
        request_id (int): The ID for the next request.
        dump (bool): Flag to enable or disable dumping of data.
        dump_dir (Path or None): Directory to store dumped data if dump is enabled.
    Methods:
        __init__(host, port, dump=False, dump_dir="dump"): Initialize the Slamtec connection.
        disconnect(): Close the socket connection.
        check_connection(): Check the connection to the Slamtec Mapper and attempt to reconnect if lost.
        _send_request(command, args=None): Send a request to the Slamtec Mapper.
        get_known_area(): Get the known area from the Slamtec Mapper.
        get_pose(): Get the current pose of the robot.
        get_map_data(): Get the map data from the Slamtec Mapper.
        _decompress_rle(b64_encoded): Decompress RLE-encoded data.
        get_laser_scan(valid_only=False): Get the laser scan data from the Slamtec Mapper.
        get_update(): Get the update status from the Slamtec Mapper.
        get_localization(): Get the localization status from the Slamtec Mapper.
        get_current_action(): Get the current action of the robot.
        get_robot_config(): Get the robot configuration.
        get_binary_config(): Get the binary configuration.
        get_robot_features_info(): Get the robot features information.
        get_sdp_version(): Get the SDP version.
        get_device_info(): Get the device information.
        set_localization(state): Set the localization state.
        set_update(state): Set the update state.
        clear_map(): Clear the map data.
        get_all(): Get all available data from the Slamtec Mapper."""

    def __init__(self, host, port, dump=False, dump_dir="dump"):
        """
        Initialize the Slamtec connection.
        Args:
            host (str): The hostname or IP address of the Slamtec device.
            port (int): The port number to connect to on the Slamtec device.
            dump (bool, optional): Flag to enable or disable dumping of data, defaults to False.
            dump_dir (str, optional): Directory to store dumped data if dump is enabled, defaults to "dump".

        Raises:
            socket.timeout: If the connection times out.
            socket.error: If there is an error connecting to the socket.

        Example:
            >>> slamtec = Slamtec("192.168.1.1", 8080, dump=True, dump_dir="data_dump")
        """

        print(f"Connecting to {host}:{port}")
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(5)
        self.socket.connect((self.host, self.port))
        self.request_id = 0
        self.dump = dump
        if self.dump:
            self.dump_dir = Path(f"{dump_dir}/{int(time.time())}/")
            self.dump_dir.mkdir(parents=True)
        else:
            self.dump_dir = None
        print("Connected")

    def disconnect(self):
        """
        Close the socket connection.
        """
        self.socket.close()

    def check_connection(self):
        """
        Check the connection to the Slamtec Mapper.

        If the connection is lost, attempt to reconnect.
        """
        # print("Checking connection")
        try:
            self.socket.settimeout(1)
            self.get_device_info()
            # If we cant data, it has disconnected
        except BlockingIOError:
            return 0    # socket is open and reading from it would block
        except:
            print("Disconnected. Reconnecting...")
            # If this fails, the exception will propogate and end execution
            self.socket.connect((self.host, self.port))
        finally:
            self.socket.settimeout(5)
            # Reset timeout
        return 0

    def _send_request(self, command, args=None):
        """
        Send a request to the Slamtec Mapper.

        Args:
            command: The command to send.
            args: Optional arguments for the command.
        
        Returns: 
            The result of the command.
        """
        request = {
            "command": command,
            "args": args,
            "request_id": self.request_id
        }
        self.request_id += 1
        data = json.dumps(request)
        # print("Sent:     {}".format(data))
        if self.dump:
            p = Path(f"{self.dump_dir}/{request['command']}-request.json")
            p.write_text(json.dumps(request, indent=2))

        data_ascii = [ord(character) for character in data]
        data_ascii.extend([10, 13, 10, 13, 10])

        # Connect to server and send data
        self.socket.sendall(bytearray(data_ascii))
        received = b""
        while True:
            size = 1024
            # Receive data from the server and shut down
            response = self.socket.recv(size)

            received += response
            if response[-4:] == b"\r\n\r\n":
                break

        received = received.decode("utf-8")
        received_json = json.loads(received)
        if type(received_json["result"]) == str:
            received_json["result"] = json.loads(received_json["result"])

        if self.dump:
            p = Path(f"{self.dump_dir}/{request['command']}-response.json")
            p.write_text(json.dumps(received_json, indent=2))

        if received_json["request_id"] != request["request_id"]:
            print("wrong request_id in response (%s != %s)" % (received_json["request_id"], request["request_id"]))
            print(received_json)
            return False
        if "code" in received_json["result"] and received_json["result"]["code"] != 1:
            print(received_json["result"].keys())
            print("command %s failed" % command)
            print(received_json)
            return False
        return received_json["result"]

    def get_known_area(self):
        """
        Get the known area from the Slamtec Mapper.

        Returns:
            The known area.
        """
        # {"args":{"kind":0,"partially":false,"type":0},"command":"getknownarea","request_id":726532096}
        response = self._send_request(command="getknownarea", args={"kind": 0, "partially": False, "type": 0})
        return response

    def get_pose(self):
        """
        Get the current pose of the robot.

        Returns:
            The current pose.
        """
        response = self._send_request(command="getpose")
        return response

    def get_map_data(self):
        """
        Get the map data from the Slamtec Mapper.

        Returns:
            The map data.
        """
        # {"args":{"area":{"height":3.750,"width":10.55000019073486,"x":-2.899999856948853,"y":-2.599999904632568},"kind":0,"partially":false,"type":0},"command":"getmapdata","request_id":3539992577}
        known_area = self.get_known_area()
        args = {
            "area": {"height": known_area["max_y"] - known_area["min_y"],
                     "width": known_area["max_x"] - known_area["min_x"],
                     "x": known_area["min_x"],
                     "y": known_area["min_y"]},
            "kind": 0,
            "partially": False,
            "type": 0
        }
        response = self._send_request(command="getmapdata", args=args)
        decompressed = self._decompress_rle(response["map_data"])

        pos = 0
        line = 1
        data_2d = {}
        while pos < len(decompressed):
            if line not in data_2d:
                data_2d[line] = []
            data_2d[line].append(decompressed[pos])

            if (pos + 1) % response['dimension_x'] == 0:
                # print(line, data_2d[line])
                line += 1
            pos += 1
        # print(line, data_2d[line])
        response["map_data"] = data_2d
        return response

    def _decompress_rle(self, b64_encoded):
        """
        Decompress RLE-encoded data.

        Args:
            b64_encoded: Base64-encoded RLE data.
        Returns:
            Decompressed data.
        """
        rle = base64.b64decode(b64_encoded)
        if rle[0:3] != b"RLE":
            print("wrong header %s" % str(rle[0:3]))
            return
        sentinel_list = [rle[3], rle[4]]

        # print("Sentinel list: %s" % sentinel_list)
        pos = 9
        decompressed = []
        while pos < len(rle):
            b = rle[pos]
            # print(b, end=", ")
            if b == sentinel_list[0]:
                # print("sentinel %i, next %i -> %i" % (sentinel_list[0], rle[pos+1], rle[pos+2] ), end=" - ")
                if rle[pos + 1] == 0 and rle[pos + 2] == sentinel_list[1]:
                    sentinel_list.reverse()
                    # print("new sentinel %s" % sentinel_list[0])
                    pos += 2
                else:
                    more = [rle[pos + 2] for i in range(rle[pos + 1])]
                    # print("adding %i" % len(more), end=" - ")
                    decompressed.extend(more)
                    pos += 2
                # break
                # print("")
            else:
                decompressed.append(b)
            pos += 1
        return decompressed

    def get_laser_scan(self, valid_only=False):
        """
        Get the laser scan data from the Slamtec Mapper.

        Args:
            valid_only (bool): If True, only return valid data points.
        Returns:
            The laser scan data.
        """
        response = self._send_request(command="getlaserscan")
        decompressed = bytearray(self._decompress_rle(response["laser_points"]))

        pos = 0
        bytes_per_row = 12
        data = []
        while pos < len(decompressed):
            parts = struct.unpack("f f h h", decompressed[pos:pos + bytes_per_row])
            pos += bytes_per_row
            distance = parts[0]
            angle_radian = parts[1]
            # todo: decode the remaining bytes
            if distance == 100000.0:
                if valid_only:
                    continue
                valid = False
            else:
                valid = True
            # print(f"distance: {distance:.4f}m, angle {math.degrees(angle_radian):.2f}°, valid {valid}")
            data.append((angle_radian, distance, valid))
            pos += bytes_per_row

        return data

    def get_update(self):
        """
        Get the update status from the Slamtec Mapper.
        """
        request = {"args": {"kind": 0}, "command": "getupdate", "request_id": 1651574155}

    def get_localization(self):
        """
        Get the localization status from the Slamtec Mapper.

        Returns:
            The localization status.
        """
        return self._send_request(command="getlocalization")

    def get_current_action(self):
        """
        Get the current action of the robot.

        Returns:
            The current action.
        """
        return self._send_request(command="getcurrentaction")

    def get_robot_config(self):
        """
        Get the robot configuration.

        Returns:
            The robot configuration.
        """
        return self._send_request(command="getrobotconfig")

    def get_binary_config(self):
        """
        Get the binary configuration.

        Returns:
            The binary configuration.
        """
        return self._send_request(command="getbinaryconfig")

    def get_robot_features_info(self):
        """
        Get the robot features information.

        Returns:
            The robot features information.
        """
        return self._send_request(command="getrobotfeaturesinfo")

    def get_sdp_version(self):
        """
        Get the SDP version.

        Returns:
            The SDP version.
        """
        return self._send_request(command="getsdpversion")

    def get_device_info(self):
        """
        Get the device information.

        Returns:
            The device information.
        """
        return self._send_request(command="getdeviceinfo")

    def set_localization(self, state):
        """
        Set the localization state.

        Args:
            param state (bool): True to enable localization, False to disable.
        """
        # True: localization on
        # False: localization off
        response = self._send_request(command="setlocalization", args={"value": state})
        # -> {"command":"setlocalization","request_id":1722096331,"result":{"code":1,"timestamp":4925591}}

    def set_update(self, state):
        """
        Set the update state.

        Args:
            param state (bool): True to enable updates, False to disable.
        """
        response = self._send_request(command="setupdate", args={"kind": 0, "value": state})
        # -> {"command":"setupdate","request_id":1722207937,"result":{"code":1,"timestamp":5751061}}

    def clear_map(self):
        """
        Clear the map data.
        """
        response = self._send_request(command="clearmap", args=0)
        # -> {"command":"clearmap","request_id":1722209408,"result":{"code":1,"timestamp":5761982}}

    def get_all(self):
        """
        Get all available data from the Slamtec Mapper.
        """
        self.get_known_area()
        self.get_pose()
        self.get_map_data()
        self.get_laser_scan()
        self.get_localization()
        self.get_current_action()
        self.get_robot_config()
        self.get_binary_config()
        self.get_robot_features_info()
        self.get_sdp_version()
        self.get_device_info()


def show_summary(st):
    """
    Show a summary of the map and localization information.

    Args:
        st: The SlamtecMapper instance.
    """
    print("Fetching Map Info...")
    known_area = st.get_known_area()
    map_data = st.get_map_data()
    print(
        f"> Map Area: ({known_area['min_x']:.4f},{known_area['min_y']:.4f},{known_area['max_x']:.4f},{known_area['max_y']:.4f})")
    print(f"> Cell Dimension: ({map_data['dimension_x']}, {map_data['dimension_y']})")
    print(f"> Cell Resolution: ({map_data['resolution']:.4f}, {map_data['resolution']:.4f})")
    print(
        f"> Cell Dimension: ({map_data['dimension_x'] * map_data['resolution']:.2f}m, {map_data['dimension_y'] * map_data['resolution']:.2f}m)")

    print("Fetching Localization Info...")
    pose = st.get_pose()
    print(f"> Position: (x {pose['x']:.4f},y {pose['y']:.4f}, z{pose['z']:.4f})")
    print(f"> Heading: {pose['yaw'] * 180.0 / math.pi:.4f}°")


def show_map(map_data):
    """
    Display the map data as an image.

    Args:
        map_data: The map data.
    """
    from PIL import Image
    scale = 4
    img = Image.new('L', (map_data['dimension_x'], map_data['dimension_y']), "black")
    pixels = img.load()
    for x in range(img.size[1]):
        for y in range(img.size[0]):
            value = map_data["map_data"][map_data['dimension_y'] - x][y]
            pixels[y, x] = value  # 0-255

    scaled_img = img.resize((map_data['dimension_x'] * scale, map_data['dimension_y'] * scale), Image.ANTIALIAS)
    scaled_img.show()


if __name__ == '__main__':
    host = "192.168.11.1"
    # host = "192.168.123.234"
    # host = "127.0.0.1"
    st = SlamtecMapper(host=host, port=1445, dump=True)
    # show_summary(st)

    """
    data = st.get_laser_scan(valid_only=False)
    csv = []
    for angle, distance, valid in data:
        csv.append(f"{angle},{distance},{math.degrees(angle)}")
    p = Path("../../laser-full.csv")
    p.write_text("\n".join(csv))
    """
    # """
    # st.get_all()
    # map_data = st.get_map_data()
    # p = Path("./map_data.txt")
    # p.write_text(map_data)
    # show_map(map_data)

    # laser_data = st.get_laser_scan()
    # show_summary(st)

    print("\n---Clearing Map---\n")
    print(st.clear_map())

    print("\n---SetLocalization---\n")
    print(st.set_localization(True))

    print("\n---Localization--- \n")
    print(st.get_localization())

    print("\n---Pose---\n")
    print(st.get_pose())

    # """
    if "--clear-map" in sys.argv:
        st.clear_map()
    if "--stop-update" in sys.argv:
        st.set_update(False)
    if "--start-update" in sys.argv:
        st.set_update(True)

    st.disconnect()
