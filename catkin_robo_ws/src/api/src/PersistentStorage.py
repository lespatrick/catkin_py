#! /usr/bin/env python
import jsonpickle
import rospy

from Location import Location

class PersistentStorage:
    locationsFileName = '/home/leszek/locations'
    locations = []

    def addLocation(self, pose, name):
        location = Location(pose, name)
        self.locations.append(location)

    def saveLocations(self):
        rospy.loginfo("writting locations")
        try:
            with open(self.locationsFileName, 'w') as locationsFile:
                locationsFile.write(self.locationsJson())
        except IOError as e:
            pass

    def readLocations(self):
        try:
            with open(self.locationsFileName) as locationsFile:
                locationsJson = locationsFile.read()
                rospy.loginfo('loaded locations')
                self.locations = jsonpickle.decode(locationsJson)
        except IOError as e:
            pass

    def locationsJson(self):
        return jsonpickle.encode(self.locations)

    def __init__(self):
        self.readLocations()
        pass
