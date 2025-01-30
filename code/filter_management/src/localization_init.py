#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from std_msgs.msg import String
from coordinate_transformation import CoordinateTransformer
from xml.etree import ElementTree as eTree


class LocaliazationInit(CompatibleNode):
    def __init__(self):
        super().__init__("localization_init")
        self.map_sub = self.new_subscription(
            String,
            "/carla/hero/OpenDRIVE",
            self.get_geoRef,
            qos_profile=1,
        )

    def get_geoRef(self, opendrive: String):
        """_summary_
        Reads the reference values for lat and lon from the carla OpenDriveMap
        Args:
            opendrive (String): OpenDrive Map from carla
        """

        root = eTree.fromstring(opendrive.data)
        header = root.find("header")
        geoRefText = header.find("geoReference").text

        self.loginfo(geoRefText)

        latString = "+lat_0="
        lonString = "+lon_0="

        indexLat = geoRefText.find(latString)
        indexLon = geoRefText.find(lonString)

        indexLatEnd = geoRefText.find(" ", indexLat)
        indexLonEnd = geoRefText.find(" ", indexLon)

        latValue = float(geoRefText[indexLat + len(latString) : indexLatEnd])
        lonValue = float(geoRefText[indexLon + len(lonString) : indexLonEnd])
        # latValue
        # lonValue

        # TODO It is 0, 0 for the maps but this should be sent to the navsat as a datam


def main(args=None):
    roscomp.init("localization_init", args=args)

    try:
        node = LocaliazationInit()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
