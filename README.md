# vision_pkg

## Image detection
### Plane side
The camera on the plane takes pictures at the rate of 15 Hz. The camera we are using, the Pointgrey Chameleon 3, has drivers provided by the manufacturer in the `pointgrey_camera_driver` package. These images are published raw and uncompressed on the `image_raw` topic. Several other topics are published to by the driver, including compressed image topics and dynamic parameter topics. `image_stamper.py` subscribes to this topic and the `state` topic, which provides current state information of the plane (location, heading, etc). It then creates a message with both the raw image and relevant state information and publishes it to the `state_image` topic at the rate of 1 Hz.

This whole system begins by running `roslaunch sniper_cam onboard_camera.launch` in the correct workspace on the Odroid. If you don't want the imaging system to terminate if your ssh session is interrupted, use `nohup roslaunch sniper_cam onboard_camera.launch &`. This launches the processes in the background and catches the hangup signal. For the script to work, the following environment variables need to be set on the Odroid:
* `ROS_IP=Odroid's IP`
* `ROS_MASTER_URI=Odriod's (http://)IP:11311`

The launchfile will also launch a node to store images taken locally, on a usb drive. The drive should be mounted with `sudo mount /dev/sda1 /mnt -o umask=000` before the flight.
Settings to adjust the white balance and color profiles are also in this launchfile. This is the extent of the image processing running on the plane.

### Ground side
To run the ground image detection software:
1. Ensure that your local environment variables are set to the following:
* `ROS_IP=Your Computer's IP`
* `ROS_MASTER_URI=Odroid's (http://)IP:11311`

An easy way to do this is to run `source .rosrc`, where `.rosrc` is the script in the root folder of this project. Make sure to edit this script first to use the proper IP addresses or make your IP address static in the router DHCP setup.

2. Ensure that you are connected to the same network as the Odroid.
3. If this is your first time running this, the proper folders need to be set up to hold images. This can be done by running the following script from the root folder of this project: `.setup_imaging_gs`.
4. Run the command `roslaunch sniper_cam manual_imaging.launch`.

A GUI should open up showing an image count in the upper left hand corner incrementing as new images are saved. Here's what's going on:
* `state_image_writer.py` subscribes to the `state_image` topic. It takes the image portion of the message and saves it to `~/Desktop/vision_files/all_images` with a filename consisting of the timestamp.jpg. The state portion of the message is written the `~/Desktop/vision_files/all_state_files` folder with a filename of the same timestamp. The format is CSV.
* `sniper_geo_locator.py` reads images from the `all_images` folder. When you left click a spot on the image, it parses the state file and figures out the GPS location of the click point, copying the image to a `target_[number]` directory, and appends the located point to a file in `target_locations_sorted` directory.
* You cycle to another target with a right mouse click. 

## Image classification and localization
All classification and localization takes place on the ground. If this is done on a different computer (and it probably should be) than the one that runs the detection (see above), the `vision_files` folder should be network shared to this computer. The basic flow is as follows:
* Run `rosrun sniper_cam image_gui.py`. This is a Tkinter application that lets you rotate, add characteristics to, and submit an image to the interopt client. 
* It shows tiled all of the images selected for a certain target. Pick the best by double clicking. It should be surrounded by a red border and fill the screen. Click and drag a box around object and select crop. Add characteristics.
* Click submit. This creates a message with all the data and the cropped, rotated image. This message is published on the `plans` (as in Death Star plans) topic. The [interopt client](https://github.com/BYU-AUVSI/interop_pkg) will read off this and send to the judge's interopt server.

## Adjusting camera settings on the fly
Two major problems were experienced at last year's competition relating to imaging: getting sufficient images to the ground and white balance on those images. A little digging helped us discover how to change parameters on the fly:
* The detection gui isn't the best way to monitor image quality as it may lag far behind the current incoming stream. We need some way to view the livestream coming in. [Image pipeline](https://github.com/ros-perception/image_pipeline) lets you subscribe to compressed or uncompressed image topics and view them raw. Run it with `rosrun image_view image_view image:=<image topic> [image transport type]` after cloning and building it with the rest of the packages.
* The pointgrey_camera_driver package publishes both compressed and uncompressed topics, so make sure you subscribe to the right one.
* [Dynamic reconfiguration](http://wiki.ros.org/dynamic_reconfigure) is a common way of dynamically changing parameters in ROS. pointgrey_camera_driver provides several nodes that support it. Run `rosrun rqt_reconfigure rqt_reconfigure` to launch the gui. From here you can change camera parameters and also compression parameters, to dynamically change streaming characteristics.


## Getting it all together
There should be 4 computers running on the ground during a flight:
1. The ground station running the [ground station package](https://github.com/BYU-AUVSI/GroundStation). 
2. The [image detection](#image-detection).
3. The [image classification and localization](#image-classification-and-localization).
4. The [interopt client](https://github.com/BYU-AUVSI/interop_pkg).
