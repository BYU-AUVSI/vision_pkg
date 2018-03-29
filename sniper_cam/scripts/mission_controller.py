#! /usr/bin/env python
from Tkinter import *
import rospy
import roslib
import signal
from uav_msgs import GetMissionWithId

PAUSE = "Pause"
RESUME = "Resume"

# Extend the tkinter class Frame to adapt to our current application
class Application(Frame):
    def __init__(self, master):
        self.master = master
        master.title("Mission Controller")
        rospy.init_node('mission_controller', anonymous=True)

        # Create the frame
        Frame.__init__(self, master)

        # Setup service
        rospy.wait_for_service("get_mission_with_id")
        self.getMissionWithId = rospy.ServiceProxy("get_mission_with_id", GetMissionWithId)

        self.pauseResumeState = PAUSE # Button switches between sending pause or resume messages

        # Add all the buttons
        self.createWidgets()
        self.master.grab_set()
        self.master.grab_release()

    def generateTapped(self):
        pass

    def sendTapped(self):
        pass

    def cancelTapped(self):
        pass

    def restartTapped(self):
        pass
    
    def pauseResumeTapped(self):
        
        self.switchPauseResumeState()
        pass

    def switchPauseResumeState(self):
        self.pauseResumeState = PAUSE if self.pauseResumeState == RESUME else RESUME
        self.pauseResumeButton.configure(text=self.pauseResumeState)

    def createWidgets(self):
        self.mpSection = LabelFrame(self.master, text="Mission Planning", pady=20)
        self.mpSection.grid(padx=10, pady=10)

        self.missionId = StringVar()
        self.missionId.set("waypoint")
        self.chosenMissionDropdown = OptionMenu(self.mpSection, self.missionId, "waypoint", "bottle_drop", "target_search", "land")
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

# create the application and run it
root = Tk()
app = Application(master=root)
app.mainloop()
