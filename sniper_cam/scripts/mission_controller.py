#! /usr/bin/env python
from Tkinter import *
from ScrolledText import *
import rospy
import roslib
import signal
import uav_msgs.msg
import uav_msgs.srv

PAUSE = "Pause"
RESUME = "Resume"

WAYPOINT_MISSION_NAME = "waypoint"
BOTTLE_DROP_MISSION_NAME = "bottle drop"
TARGET_SEARCH_MISSION_NAME = "target search"
OTHER_MISSION_NAME = "other"
LAND_MISSION_NAME = "land"

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

        # First, check if we need to generate points now
        if mission_name == TARGET_SEARCH_MISSION_NAME:
            search_area_bounds = self.getMissionWithId(mission_id)
            # Now generate a search path

        elif mission_name in INTEROP_SOURCE:
            result = self.getMissionWithId(mission_id)
            mission_to_generate = result # Pass straight through
            self.setDataLabel(result)

        else:
            # Read points from file
            pass


    def sendTapped(self):
        pass

    def cancelTapped(self):
        pass

    def restartTapped(self):
        pass
    
    def pauseResumeTapped(self):
        self.switchPauseResumeState() 

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

        #print("Waiting for 'send_waypoints' service to become available")
        #rospy.wait_for_service("send_waypoints")
        self.uploadPath = rospy.ServiceProxy("send_waypoints", uav_msgs.srv.UploadPath)

        #print("Waiting for 'plan_mission' service to become available")
        #rospy.wait_for_service("plan_mission")
        self.generatePath = rospy.ServiceProxy("plan_mission", uav_msgs.srv.GeneratePath)

    def createWidgets(self):
        self.mpSection = LabelFrame(self.master, text="Mission Planning", pady=20)
        self.mpSection.grid(padx=10, pady=10)

        self.missionId = StringVar()
        self.missionId.set(MISSION_TYPES[0])
        self.chosenMissionDropdown = OptionMenu(self.mpSection, self.missionId, *MISSION_TYPES)
        self.chosenMissionDropdown.configure(width=10)
        self.chosenMissionDropdown.grid(row=0, column=0, padx=10, pady=10)

        self.generatePathButton = Button(self.mpSection, text="Generate Path", command=self.generateTapped, state=NORMAL)
        self.generatePathButton.grid(row=0, column=1, padx=10, pady=10)

        self.sendPathButton = Button(self.mpSection, text="Send Path", command=self.sendTapped, state=DISABLED)
        self.sendPathButton.grid(row=0, column=2, padx=10, pady=10)


        self.mcSection = LabelFrame(self.master, text="Mission Control", pady=10)
        self.mcSection.grid(padx=10, pady=10)

        self.statusLabel = Label(self.mcSection, text="Status")
        self.statusLabel.grid(row=0, column=0, padx=10, pady=10)

        self.pauseResumeButton = Button(self.mcSection, width=10, text=self.pauseResumeState, command=self.pauseResumeTapped, state=NORMAL)
        self.pauseResumeButton.grid(row=1, column=0, padx=10, pady=10)

        self.cancelButton = Button(self.mcSection, text="Cancel", command=self.cancelTapped, state=NORMAL)
        self.cancelButton.grid(row=1, column=1, padx=10, pady=10)

        self.restartButton = Button(self.mcSection, text="Restart", command=self.restartTapped, state=NORMAL)
        self.restartButton.grid(row=1, column=2, padx=10, pady=10)


        self.dataSection = LabelFrame(self.master, text="Data", pady=20)
        self.dataSection.grid(padx=10, pady=10)

        self.dataLabel = ScrolledText(root, width=50, height=20)
        self.dataLabel.grid(padx = 10, pady=10)


# create the application and run it
root = Tk()
app = Application(master=root)
app.mainloop()
