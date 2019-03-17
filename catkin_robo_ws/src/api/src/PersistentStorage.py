#! /usr/bin/env python
import jsonpickle

from src.model.Location import Location


class PersistentStorage:
    locationsFileName = 'locations'
    locations = []

    def __init__(self):
        pass

    def addLocation(self, pose, name):
        location = Location(pose, name)
        self.locations.add(location)

    def saveLocations(self):
        locationsJson = jsonpickle.encode(self.locations)
        locationsFile = open(self.locationsFileName, 'w')
        locationsFile.write(locationsJson)
        locationsFile.close()

    def readLocations(self):
        try:
            with open(self.locationsFileName) as locationsFile:
                locationsJson = locationsFile.read()
                self.locations = jsonpickle.decode(locationsJson)
        except IOError as e:
            pass

    def locationsJson(self):
        return jsonpickle.encode(self.locations)
