#!/usr/bin/env python

#  Class used to create a data structure based on camera pose topic


class Coordinates(object):
    def __init__(self, time0, x0, y0, z0):
        """Create new Coordinates structure
        Keyword arguments:
        time0 -- time stamp in seconds (float)
        x0 -- coordinate x (as float)
        y0 -- coordinate y (as float)
        z0 -- coordinate z (as float)
        """
        self.timeSeconds = time0
        self.x = x0
        self.y = y0
        self.z = z0

    def get_time(self):
        """
        return: time in seconds
        """
        return self.timeSeconds

    def get_coordinate_x(self):
        """
        return: Coordinate X
        """
        return self.x

    def get_coordinate_y(self):
        """
        return: Coordinate Y
        """
        return self.y

    def get_coordinate_z(self):
        """
        return: Coordinate Z
        """
        return self.z
