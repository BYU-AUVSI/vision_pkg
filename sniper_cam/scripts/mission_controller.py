#! /usr/bin/env python
from Tkinter import *
from ScrolledText import *
from tkFileDialog import askopenfilename

import rospy
import roslib
import signal
import uav_msgs.msg
import uav_msgs.srv
from rosplane_msgs.msg import *

# Use for lawnmower path geometry
import shapely.geometry
import pyproj

PAUSE = "Pause"
RESUME = "Resume"

WAYPOINT_MISSION_NAME = "waypoint"
BOTTLE_DROP_MISSION_NAME = "bottle drop"
TARGET_SEARCH_MISSION_NAME = "target search"
OTHER_MISSION_NAME = "other"
LAND_MISSION_NAME = "land"

INIT_STATE = "Nothing planned or sent yet"
PATH_GENERATED_STATE = "Planned path"
PATH_SEND_STATE = "Path sent to plane"

MISSION_TYPES = (WAYPOINT_MISSION_NAME, BOTTLE_DROP_MISSION_NAME, TARGET_SEARCH_MISSION_NAME, OTHER_MISSION_NAME, LAND_MISSION_NAME)

mission_type_dict = {
    WAYPOINT_MISSION_NAME: uav_msgs.msg.JudgeMission.MISSION_TYPE_WAYPOINT,
    BOTTLE_DROP_MISSION_NAME: uav_msgs.msg.JudgeMission.MISSION_TYPE_DROP,
    TARGET_SEARCH_MISSION_NAME: uav_msgs.msg.JudgeMission.MISSION_TYPE_SEARCH,
    OTHER_MISSION_NAME: uav_msgs.msg.JudgeMission.MISSION_TYPE_OTHER,
    LAND_MISSION_NAME: uav_msgs.msg.JudgeMission.MISSION_TYPE_LAND,
}

INTEROP_SOURCE = (WAYPOINT_MISSION_NAME, BOTTLE_DROP_MISSION_NAME, TARGET_SEARCH_MISSION_NAME) # These mission types require talking to interop

# Extend the tkinter class Frame to adapt to our current application
class Application(Frame):
    def __init__(self, master):
        self.master = master
        master.title("Mission Controller")
        rospy.init_node('mission_controller', anonymous=True)

        # Create the frame
        Frame.__init__(self, master)

        # Setup services
        self.setupServices()

        self.pauseResumeState = PAUSE # Button switches between sending pause or resume messages

        # Add all the buttons
        self.createWidgets()
        self.master.grab_set()
        self.master.grab_release()

    def generateTapped(self):
        mission_name, mission_id = self.getDropdownData()
        interop_data = self.getMissionWithId(mission_id).mission # If mission id not valid, will return everything but waypoints

        # First, check if we need to generate waypoints
        if mission_name == TARGET_SEARCH_MISSION_NAME:
            # Now generate a search path
            points = self.generateLawnmowerPath(interop_data)
            new_waypoints = []
            i = 1

            # Transform shapely points into JudgeMission OrderedPoints
            for point in points:
                op = uav_msgs.msg.OrderedPoint()
                op.ordinal = i

                p = uav_msgs.msg.Point()
                p.latitude = point.y
                p.longitude = point.x
                p.altitude = 40
                op.point = p

                new_waypoints.append(op)

                i += 1
            
            # Replace waypoints from judges (which in this case are actually the search area points) with the generated
            # lawnmower path points.
            interop_data.waypoints = new_waypoints

        elif mission_name in INTEROP_SOURCE:
            # Send to mission_planner immediately
            pass

        else:
            # Read points from file
            filename = askopenfilename(title="Select file of waypoints") # show an "Open" dialog box and return the path to the selected file
            if not filename:
                return

            with open(filename) as f:
                lines = f.readlines()
                i = 0
                for line in lines:
                    data = line.split(',')
                    data = [float(d.strip()) for d in data]
                    p = uav_msgs.msg.Point()
                    p.latitude = data[0]
                    p.longitude = data[1]
                    p.altitude = data[2]

                    op = uav_msgs.msg.OrderedPoint()
                    op.point = p
                    op.ordinal = i
                    interop_data.waypoints.append(op)

                    i += 1

        self.setDataLabel(interop_data)
        self.generatePath(interop_data)
        self.setState(PATH_GENERATED_STATE)
            
    def sendTapped(self):
        self.uploadPath()
        self.setState(PATH_SEND_STATE)

    def cancelTapped(self):
        pass

    def restartTapped(self):
        pass
    
    def pauseResumeTapped(self):
        self.switchPauseResumeState() 

    def setState(self, newState):
        self.state = newState
        self.statusLabel.configure(text=self.state)
        
        if self.state == INIT_STATE:
            self.generatePathButton.configure(state=NORMAL)
            self.sendPathButton.configure(state=DISABLED)
        elif self.state == PATH_GENERATED_STATE:
            self.generatePathButton.configure(state=NORMAL)
            self.sendPathButton.configure(state=NORMAL)
        elif self.state == PATH_SEND_STATE:
            self.generatePathButton.configure(state=NORMAL)
            self.sendPathButton.configure(state=NORMAL)

    def setDataLabel(self, data):
        self.dataLabel.configure(state=NORMAL)
        self.dataLabel.delete(1.0,END)
        self.dataLabel.insert(END, data)
        self.dataLabel.configure(state=DISABLED)

    # Returns tuple of visible string, id
    def getDropdownData(self):
        mission_string = self.missionId.get()
        return (mission_string, mission_type_dict[mission_string])

    def switchPauseResumeState(self):
        self.pauseResumeState = PAUSE if self.pauseResumeState == RESUME else RESUME
        self.pauseResumeButton.configure(text=self.pauseResumeState)

    def setupServices(self):
        print("Waiting for 'get_mission_with_id' service from interop client.py to become available")
        rospy.wait_for_service("get_mission_with_id")
        self.getMissionWithId = rospy.ServiceProxy("get_mission_with_id", uav_msgs.srv.GetMissionWithId)
        print("Found service\n")

        print("Waiting for 'send_waypoints' service to become available")
        rospy.wait_for_service("theseus/send_waypoints")
        self.uploadPath = rospy.ServiceProxy("theseus/send_waypoints", uav_msgs.srv.UploadPath)
        print("Found service\n")

        print("Waiting for 'plan_mission' service to become available")
        rospy.wait_for_service("theseus/plan_mission")
        self.generatePath = rospy.ServiceProxy("theseus/plan_mission", uav_msgs.srv.GeneratePath)
        print("Found service\n")

    def createWidgets(self):
        self.mpSection = LabelFrame(self.master, text="Mission Planning", pady=20)
        self.mpSection.grid(padx=10, pady=10)

        self.missionId = StringVar()
        self.missionId.set(MISSION_TYPES[0])
        self.chosenMissionDropdown = OptionMenu(self.mpSection, self.missionId, *MISSION_TYPES)
        self.chosenMissionDropdown.configure(width=10)
        self.chosenMissionDropdown.grid(row=0, column=0, padx=10, pady=10)

        self.generatePathButton = Button(self.mpSection, text="Generate Path", command=self.generateTapped)
        self.generatePathButton.grid(row=0, column=1, padx=10, pady=10)

        self.sendPathButton = Button(self.mpSection, text="Send Path", command=self.sendTapped)
        self.sendPathButton.grid(row=0, column=2, padx=10, pady=10)


        self.mcSection = LabelFrame(self.master, text="Mission Control", pady=10)
        self.mcSection.grid(padx=10, pady=10)

        self.statusText = Label(self.mcSection, text="Status")
        self.statusText.grid(row=0, column=0, padx=10, pady=10)

        self.statusLabel = Label(self.mcSection)
        self.statusLabel.grid(row=0, column=1, padx=10, pady=10)
        self.setState(INIT_STATE)


        self.pauseResumeButton = Button(self.mcSection, width=10, text=self.pauseResumeState, command=self.pauseResumeTapped, state=DISABLED)
        self.pauseResumeButton.grid(row=1, column=0, padx=10, pady=10)

        self.cancelButton = Button(self.mcSection, text="Cancel", command=self.cancelTapped, state=DISABLED)
        self.cancelButton.grid(row=1, column=1, padx=10, pady=10)

        self.restartButton = Button(self.mcSection, text="Restart", command=self.restartTapped, state=DISABLED)
        self.restartButton.grid(row=1, column=2, padx=10, pady=10)


        self.dataSection = LabelFrame(self.master, text="Data", pady=20)
        self.dataSection.grid(padx=10, pady=10)

        self.dataLabel = ScrolledText(root, width=50, height=20)
        self.dataLabel.grid(padx = 10, pady=10)

    def generateLawnmowerPath(self, mission):
        # Set up projections. We project from GPS into a flat plane to allow us to specify 
        # point spacing accurately. These newly generate points then are projected back to GPS.
        p_gps = pyproj.Proj(init='epsg:4326')
        p_flat = pyproj.Proj(init='epsg:3857') # metric; same as EPSG:900913

        # Perimeter of points
        boundaries = mission.waypoints
        perimeter = [(p.point.longitude, p.point.latitude) for p in boundaries]

        polygon = shapely.geometry.Polygon(perimeter)

        max_lat = perimeter[0][1]
        max_lon = perimeter[0][0]
        min_lat = perimeter[0][1]
        min_lon = perimeter[0][0]
        for p in perimeter:
            if p[0] > max_lon:
                max_lon = p[0]
            if p[0] < min_lon:
                min_lon = p[0]
            if p[1] > max_lat:
                max_lat = p[1]
            if p[1] < min_lat:
                min_lat = p[1]

        # Upper left and lower right corner of bounding box
        upper_left = shapely.geometry.Point((min_lon, max_lat))
        lower_right = shapely.geometry.Point((max_lon, min_lat))

        front_to_back_step_size = 100 # 1000 = 1 km grid step size
        side_to_side_step_size = 100

        # Project corners to target projection
        s = pyproj.transform(p_gps, p_flat, upper_left.x, upper_left.y) # Transform NW point to 3857
        e = pyproj.transform(p_gps, p_flat, lower_right.x, lower_right.y) # .. same for SE

        # Determine long flight direction. We want to fly across the longest axis to minimize turns.
        horiz = abs(s[0] - e[0])
        vert = abs(s[1] - e[1])

        if vert > horiz:
            forwards = True
            gridpoints = []
            x = s[0]
            while x < e[0]:
                if forwards:
                    y = s[1]
                    while y > e[1]:
                        p = shapely.geometry.Point(pyproj.transform(p_flat, p_gps, x, y))
                        gridpoints.append(p)
                        y -= front_to_back_step_size
                else:
                    y = e[1]
                    while y < s[1]:
                        p = shapely.geometry.Point(pyproj.transform(p_flat, p_gps, x, y))
                        gridpoints.append(p)
                        y += front_to_back_step_size

                forwards = not forwards
                x += side_to_side_step_size

        else:
            forwards = True
            gridpoints = []
            y = s[1]
            while y > e[1]:
                if forwards:
                    x = s[0]
                    while x < e[0]:
                        p = shapely.geometry.Point(pyproj.transform(p_flat, p_gps, x, y))
                        gridpoints.append(p)
                        x += front_to_back_step_size
                else:
                    x = e[0]
                    while x > s[0]:
                        p = shapely.geometry.Point(pyproj.transform(p_flat, p_gps, x, y))
                        gridpoints.append(p)
                        x -= front_to_back_step_size

                forwards = not forwards
                y -= side_to_side_step_size

        # Trim all points that don't lie within the polygon (boundaries of search area)
        inside = [p for p in gridpoints if p.within(polygon)]
        return inside

# create the application and run it
root = Tk()
app = Application(master=root)
app.mainloop()
