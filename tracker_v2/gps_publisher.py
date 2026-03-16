#!/usr/bin/env python3
"""
gps_publisher.py  –  ROS2 node that reads NMEA from serial and publishes
                      UTM (easting, northing) positions.

Replicates the pipeline from the donkeycar GpsPosition / GpsNmeaPositions parts.

Published topic : /gps/utm   (std_msgs/Float64MultiArray)
                  data[0] = UTM easting  (metres, i.e. "longitude" in donkeycar)
                  data[1] = UTM northing (metres, i.e. "latitude"  in donkeycar)
                  data[2] = timestamp (unix float, same as donkeycar ts)

Parameters (ros2 param or launch args):
  serial_port   (str)   default '/dev/ttyUSB0'
  baud_rate     (int)   default 9600
  timeout_sec   (float) default 0.5
  debug         (bool)  default False
"""

import functools
import operator
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import serial
import utm


# ──────────────────────────────────────────────────────────────
#  Pure parsing helpers  (direct port from donkeycar gps.py)
# ──────────────────────────────────────────────────────────────

def _parse_nmea_checksum(nmea_line: str) -> int:
    """Extract the checksum hex value from the end of a raw NMEA sentence."""
    return int(nmea_line[-2:], 16)


def _calculate_nmea_checksum(nmea_line: str) -> int:
    """XOR all bytes between '$' and '*' to produce the expected checksum."""
    return functools.reduce(operator.xor, map(ord, nmea_line[1:-3]), 0)


def _nmea_to_degrees(gps_str: str, direction: str) -> float:
    """
    Convert NMEA coordinate string (DDDMM.MMMMM) + cardinal direction to
    signed decimal degrees.

    The donkeycar version is replicated exactly: split on '.', take the last
    two chars of the integer part as whole minutes, everything before as
    degrees.
    """
    if not gps_str or gps_str == "0":
        return 0.0

    parts = gps_str.split(".")
    degrees_str = parts[0][:-2]          # 0–3 digit integer degrees
    minutes_str = parts[0][-2:]          # 2-digit whole minutes

    if len(parts) == 2:
        minutes_str += "." + parts[1]   # append fractional minutes

    degrees = float(degrees_str) if degrees_str else 0.0
    minutes = float(minutes_str) / 60.0 if minutes_str else 0.0

    sign = -1.0 if direction in ("W", "S") else 1.0
    return (degrees + minutes) * sign


def parse_gps_position(line: str, logger=None):
    """
    Parse a single raw NMEA line.

    Returns (utm_easting, utm_northing) as floats if the line is a valid
    GPRMC / GNRMC sentence with a good fix and matching checksum.
    Returns None otherwise (bad checksum, no fix, wrong message type, etc.).

    This is a direct port of donkeycar's parseGpsPosition().
    """
    if not line:
        return None
    line = line.strip()
    if not line:
        return None

    # Must start with '$' and have '*##' checksum at the end
    if line[0] != "$":
        if logger:
            logger.debug("NMEA missing line start '$'")
        return None
    if line[-3] != "*":
        if logger:
            logger.debug("NMEA missing checksum '*'")
        return None

    # Validate checksum
    if _parse_nmea_checksum(line) != _calculate_nmea_checksum(line):
        if logger:
            logger.debug("NMEA checksum mismatch – dropping sentence")
        return None

    # Split body (strip leading '$' and trailing '*##')
    parts = line[1:-3].split(",")
    message_type = parts[0]

    if message_type not in ("GPRMC", "GNRMC"):
        return None   # not a position sentence – silently ignore

    # Field [2]: 'A' = valid fix, 'V' = warning / no fix
    if parts[2] == "V":
        if logger:
            logger.debug("GPS receiver warning – no fix, dropping")
        return None

    # Decode position
    # parts[3]=lat, parts[4]=N/S, parts[5]=lon, parts[6]=E/W
    latitude  = _nmea_to_degrees(parts[3], parts[4])
    longitude = _nmea_to_degrees(parts[5], parts[6])

    # Convert to UTM (same library call as donkeycar)
    utm_position = utm.from_latlon(latitude, longitude)
    # utm.from_latlon returns (easting, northing, zone_number, zone_letter)

    if logger:
        logger.debug(
            f"UTM easting={utm_position[0]:.2f}  northing={utm_position[1]:.2f}"
        )

    return float(utm_position[0]), float(utm_position[1])


# ──────────────────────────────────────────────────────────────
#  ROS2 node
# ──────────────────────────────────────────────────────────────

class GpsPublisher(Node):
    """
    Reads NMEA sentences from a serial port, converts to UTM, and publishes
    on /gps/utm at whatever rate the GPS module delivers fixes (~1 Hz typical).
    """

    def __init__(self):
        super().__init__("gps_publisher")

        # ── Declare / read parameters ──────────────────────────
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baud_rate",    460800)
        self.declare_parameter("timeout_sec",  0.5)
        self.declare_parameter("debug",        True)

        port    = self.get_parameter("serial_port").value
        baud    = self.get_parameter("baud_rate").value
        timeout = self.get_parameter("timeout_sec").value
        self._debug = self.get_parameter("debug").value

        # ── Publisher ──────────────────────────────────────────
        self._pub = self.create_publisher(Float64MultiArray, "/gps/utm", 10)

        # ── Serial port ────────────────────────────────────────
        self.get_logger().info(f"Opening serial port {port} @ {baud} baud")
        self._serial = serial.Serial(port, baudrate=baud, timeout=timeout)

        # ── Timer – poll serial at ~10 Hz, GPS sentences arrive ~1 Hz ──
        # The donkeycar part reads in a tight loop; we use a timer instead
        # so the ROS2 event loop stays responsive.
        self._timer = self.create_timer(0.1, self._read_and_publish)

        self.get_logger().info("GPS publisher ready – waiting for fix...")

    # ──────────────────────────────────────────────────────────
    def _read_and_publish(self):
        """
        Read all lines currently available on the serial port and publish
        the most recent valid UTM position, mirroring donkeycar's behaviour
        of returning the last element of the positions list.
        """
        positions = []

        # Drain whatever the serial buffer currently holds
        while self._serial.in_waiting:
            try:
                raw = self._serial.readline()
                line = raw.decode("ascii", errors="replace")
            except serial.SerialException as exc:
                self.get_logger().error(f"Serial read error: {exc}")
                return

            ts = time.time()
            result = parse_gps_position(
                line,
                logger=self.get_logger() if self._debug else None,
            )
            if result is not None:
                easting, northing = result
                positions.append((ts, easting, northing))

        if not positions:
            return

        # donkeycar returns positions[-1] (most recent)
        ts, easting, northing = positions[-1]

        msg = Float64MultiArray()
        msg.data = [easting, northing, ts]
        self._pub.publish(msg)

        if self._debug:
            self.get_logger().info(
                f"Published  easting={easting:.2f}  northing={northing:.2f}"
            )

    # ──────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("Shutting down GPS publisher")
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


# ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
