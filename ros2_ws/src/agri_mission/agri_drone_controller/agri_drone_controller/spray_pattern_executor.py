#!/usr/bin/env python3
"""
Spray Pattern Generator.

Generates lawnmower waypoints for systematic zone coverage.
"""

import math
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion


class SprayPatternExecutor:
    """Generates lawnmower (boustrophedon) spray waypoints for a rectangular zone."""

    def generate_lawnmower_waypoints(
        self,
        zone_origin_x: float,
        zone_origin_y: float,
        zone_width: float,
        zone_length: float,
        altitude: float,
        spray_width: float = 4.0,
        overlap: float = 0.20,
        heading_deg: float = 0.0,
    ) -> PoseArray:
        """
        Generate lawnmower pattern waypoints.

        Args:
            zone_origin_x/y: SW corner of zone in local ENU frame (meters)
            zone_width: E-W extent (meters)
            zone_length: N-S extent (meters)
            altitude: spray altitude AGL (meters)
            spray_width: effective spray swath per pass (meters)
            overlap: fractional strip overlap (0.0-1.0)
            heading_deg: zone orientation from North (degrees), 0=N-S strips

        Returns:
            PoseArray with alternating strip endpoints
        """
        effective_width = spray_width * (1.0 - overlap)
        num_strips = max(1, int(math.ceil(zone_width / effective_width)))

        waypoints = PoseArray()
        waypoints.header.frame_id = 'map'

        for i in range(num_strips):
            x_offset = i * effective_width + effective_width / 2.0
            # Clamp to zone
            x = zone_origin_x + min(x_offset, zone_width - 0.1)

            if i % 2 == 0:  # N to S
                y_start = zone_origin_y
                y_end = zone_origin_y + zone_length
            else:            # S to N
                y_start = zone_origin_y + zone_length
                y_end = zone_origin_y

            for y in [y_start, y_end]:
                pose = Pose()
                pose.position = Point(x=x, y=y, z=altitude)
                # Yaw points along strip direction
                if i % 2 == 0:
                    yaw = 0.0  # North
                else:
                    yaw = math.pi  # South
                pose.orientation = Quaternion(
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                )
                waypoints.poses.append(pose)

        return waypoints

    def get_zone_waypoints(self, zone_id: str, zones_config: dict) -> PoseArray:
        """Look up zone from config and generate waypoints."""
        if zone_id not in zones_config:
            return PoseArray()

        zone = zones_config[zone_id]
        cx = zone['center_x']
        cy = zone['center_y']
        width = zone['width']
        length = zone['length']
        alt = zone.get('spray_altitude', 3.0)
        spray_w = zone.get('spray_width', 4.0)
        overlap = zone.get('overlap', 0.20)

        # Compute SW corner from center
        origin_x = cx - width / 2.0
        origin_y = cy - length / 2.0

        return self.generate_lawnmower_waypoints(
            origin_x, origin_y, width, length, alt, spray_w, overlap
        )
