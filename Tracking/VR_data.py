import triad_openvr
import Constants
from math import sin, cos, pi

v = None
try:
    v = triad_openvr.triad_openvr()
    v.print_discovered_objects()
except:
    print("Error Encountered")


class TrackerData:
    def __init__(self, x = 99, y = 99, z = 99, x_rot = 999, y_rot = 999, z_rot = 999, r = 1, i = 0, j = 0, k = 0):
        self.x = x
        self.y = y
        self.z = z
        self.x_rot = x_rot
        self.y_rot = y_rot
        self.z_rot = z_rot
        self.r = r
        self.i = i
        self.j = j
        self.k = k

    def convert_to_pixels(self):
        pixel_x = (1 + self.x) / 2 * Constants.display_width
        pixel_y = (1 + self.x) / 2 * Constants.display_height
        return pixel_x, pixel_y


def get_data(tracker_number):
    tracker = TrackerData()
    try:
        if v.devices["tracker_"+str(tracker_number)] is not None:
            tracker_pose_euler = v.devices["tracker_"+str(tracker_number)].get_pose_euler()
            tracker_pose_quaternion = v.devices["tracker_"+str(tracker_number)].get_pose_quaternion()
            if tracker_pose_euler is not None:
                tracker.x = round(tracker_pose_euler[0], 2)
                tracker.y = round(tracker_pose_euler[1], 2)
                tracker.z = round(tracker_pose_euler[2], 2)
                tracker.x_rot = round(tracker_pose_euler[3], 2)
                tracker.y_rot = round(tracker_pose_euler[4], 2)
                tracker.z_rot = round(tracker_pose_euler[5], 2)
                # Tracker behind robot lift
                if tracker_number == 1:
                    offset = 0.3
                    tracker.x += offset * sin(tracker.y_rot * pi / 180)
                    tracker.z -= offset * cos(tracker.y_rot * pi / 180)

                # Tracker on robot arm
                if tracker_number == 2:
                    offset = 0
                    tracker.x -= offset * cos(tracker.y_rot * pi / 180)
                    tracker.z -= offset * sin(tracker.y_rot * pi / 180)
                    print(tracker.x, tracker.z)
                tracker.r, tracker.i, tracker.j, tracker.k = tracker_pose_quaternion[3::]
    except:
        print("Unable to get data for tracker "+str(tracker_number))
    return tracker


def print_data(tracker):
    print("X = ", tracker.x)
    print("Y = ", tracker.y)
    print("Z = ", tracker.z)
    print("X_Rot = ", tracker.x_rot)
    print("Y_Rot = ", tracker.y_rot)
    print("Z_Rot = ", tracker.z_rot)
