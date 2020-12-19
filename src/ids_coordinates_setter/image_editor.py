#!/usr/bin/env python

import piexif  # need to install by pip install piexif
import exifread # need to install by pip install exifread
from fractions import Fraction
import datetime
import time

#  Class used to change image EXIF Data


def set_gps_location(file_name, lat, lng, altitude):
    """Adds GPS position as EXIF metadata
    Keyword arguments:
    file_name -- image file
    lat -- latitude (as float)
    lng -- longitude (as float)
    altitude -- altitude (as float)
    """
    lat_deg = to_deg(lat, ["S", "N"])
    lng_deg = to_deg(lng, ["W", "E"])

    exif_lat = (change_to_rational(lat_deg[0]), change_to_rational(lat_deg[1]),
                change_to_rational(lat_deg[2]))
    exif_lng = (change_to_rational(lng_deg[0]), change_to_rational(lng_deg[1]),
                change_to_rational(lng_deg[2]))

    # Create new EXIF GPS data
    gps_ifd = {
        piexif.GPSIFD.GPSAltitudeRef: 1,
        piexif.GPSIFD.GPSAltitude: change_to_rational(round(altitude, 2)),
        piexif.GPSIFD.GPSLatitudeRef: lat_deg[3],
        piexif.GPSIFD.GPSLatitude: exif_lat,
        piexif.GPSIFD.GPSLongitudeRef: lng_deg[3],
        piexif.GPSIFD.GPSLongitude: exif_lng,
    }

    gps_exif = {"GPS": gps_ifd}

    # get original exif data first!
    try:
        exif_data = piexif.load(file_name)
        # update original exif data to include GPS tag
        exif_data.update(gps_exif)
        exif_bytes = piexif.dump(exif_data)
        # Save EXIF data in image
        piexif.insert(exif_bytes, file_name)
    except:
        exif_bytes = piexif.dump(gps_exif)
        # Save EXIF data in image
        piexif.insert(exif_bytes, file_name)


def change_to_rational(number):
    """convert a number to rational
    Keyword arguments: number
    return: tuple like (1, 2), (numerator, denominator)
    """
    f = Fraction(str(number))
    return f.numerator, f.denominator


def to_deg(value, loc):
    """convert decimal coordinates into degrees, minutes and seconds tuple
    Keyword arguments: value is float gps-value, loc is direction list ["S", "N"] or ["W", "E"]
    return: tuple like (25, 13, 48.343 ,'N')
    """
    if value < 0:
        loc_value = loc[0]
    elif value > 0:
        loc_value = loc[1]
    else:
        loc_value = ""

    abs_value = abs(value)
    deg = int(abs_value)
    t1 = (abs_value-deg)*60
    min_v = int(t1)
    sec = round((t1 - min_v) * 60, 5)
    return deg, min_v, sec, loc_value


def get_image_timestamp(path):
    try:
        with open(path, 'rb') as image_file:  # open image
            target_timestamp = 0.0
            # Get Image Datetime Original
            tags = exifread.process_file(image_file, stop_tag="EXIF DateTimeOriginal")
            date_taken = tags["EXIF DateTimeOriginal"]

            # Convert to Seconds
            datetime_object = datetime.datetime.strptime(str(date_taken), '%Y:%m:%d %H:%M:%S')
            target_timestamp = time.mktime(datetime_object.timetuple())
            return target_timestamp
    except:
        return 0.0
