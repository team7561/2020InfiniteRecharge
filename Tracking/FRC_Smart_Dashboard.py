import threading
from networktables import NetworkTables
from VR_data import TrackerData
import Constants

cond = threading.Condition()
notified = [False]

def connection_listener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()
class OdometryData:
    def __init__(self, x = 99, y = 99, rot = 999):
        self.x = x
        self.y = y
        self.rot = rot

def convert_to_pixels(self):
    pixel_x = (1 + self.x) / 2 * Constants.display_width
    pixel_y = (1 + self.x) / 2 * Constants.display_height
    return pixel_x, pixel_y


NetworkTables.initialize(server=Constants.networktables_IP)
try:
    NetworkTables.addconnection_listener(connection_listener, immediateNotify=True)
except:
    print("Unable to connect to network tables")

table = NetworkTables.getTable('SmartDashboard')

def get_target_location():
    return TrackerData(table.getNumber("odometry_x", 0), 0, table.getNumber("odometry_y", 0), 0, table.getNumber("odometry_theta", 0), 0)

def get_robot_odometry():
    return OdometryData(table.getNumber("Target Location X", 0), table.getNumber("Target Location Z", 0), table.getNumber("Target Location Heading", 0))

def get_destination_location():
    return TrackerData(table.getNumber("Destination Location X", 0), 0, table.getNumber("Destination Location Z", 0), 0, table.getNumber("Destination Location Heading", 0), 0)

def publish_tracker_data(tracker, tracker_number):
    table.putNumber("x"+str(tracker_number), tracker.x)
    table.putNumber("y"+str(tracker_number), tracker.y)
    table.putNumber("z"+str(tracker_number), tracker.z)
    table.putNumber("x_rot"+str(tracker_number), tracker.x_rot)
    table.putNumber("y_rot"+str(tracker_number), tracker.y_rot)
    table.putNumber("z_rot"+str(tracker_number), tracker.z_rot)
    table.putNumber("r"+str(tracker_number), tracker.r)
    table.putNumber("i"+str(tracker_number), tracker.i)
    table.putNumber("j"+str(tracker_number), tracker.j)
    table.putNumber("k"+str(tracker_number), tracker.k)
#   print_data(x, y, z, x_rot, y_rot, z_rot)
def publish_other_data(fps, count):
    table.putNumber("count", count)
    table.putNumber("fps", fps)

def get_step():
    return table.getNumber("Vive Auto", -1)