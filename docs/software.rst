============
Software
============

This section will detail the various software components that come along with the Platypus Lutra 1.x series boats. The software can be downloaded from http://www.senseplatypus.com/software/lutra

Please read this section carefully before deploying the boat.

-------------------
Operator Interface
-------------------

The Operator Interface (OI) can run on Windows, OSX and Linux. The OI can be used to send remote commands to the boat via a local network or 3G. You can also receive a live stream of images and sensor data on the OI. 

----------------------------------------
Establishing A Connection With the Boat
----------------------------------------

The following steps describe the steps involved in connecting the Operator interface to a real and the steps involved in creating a simulated boat.

1.	Unzip the Lutra_1_x folder.

2.	Open the appropriate Operator Interface executable for your operating system from the sami-core/run/bin folder.
a.	sami-osx.command for OSX operating system
b.	sami-win32.bat for Windows with a 32-bit JVM
c.	sami-win64.bat for Windows with a 64-bit JVM

3.	First time the operator interface is open on your computer, it should ask you for the location of two files:
a.	Domain Configuration File – In the browser select “sami-crw/configs/crw.dcf 
b.	 DREAAM specification file – In the browser, select “sami-crw/plans/crw.drm

4.	At this point, several operator interface windows should have appeared, all stacked on top of each other.
5.	You can organize the windows as you like. A common way of organizing the windows is shown Fig.
	[fig]
6.	Clicking the “Save” button in the Frame Manager window will save the location of all the operator interface windows. You can auto align the windows to the saved perspective by clicking on the “Restore” button, the next time you open the operator interface. You can also close all the windows of the operator interface by clicking on the “Close” button.
	[fig]
7.	There are 5 windows in the operator interface: Frame Manager, Message Frame, Map Frame, Mission Monitor and the Operator Interaction Frame.
	[fig]
8.	To connect the operator interface to the boat, click on the “Connect Boat” option in the left panel of the Mission Monitor Window.
  	[fig]
9.	Then click on the run button in the left panel of the Mission Monitor Window.
	[fig]
10.	 In the Operator Interaction Window, enter the IP address of the phone that was displayed in the Airboat Setup app.
	[fig]
11.	Click on the “Done” button in the Operator Interaction Window.
  	[fig]
12.	The boat marker will now appear in the Map Window. 
	[fig]
13.	You can select the boat by clicking on the boat marker displayed in the Map Window.
	[fig]
	
----------------
Operating Modes
----------------

The Platypus Lutra 1.x series boats have two main modes of operation: Tele-operation mode and autonomous mode. The following subsections describe these modes in detail.

--------------------
Tele-operation Mode
--------------------

i.	The tele-operation mode gives the operator the ability to control the robot manually by varying its thrust and steering the shroud assembly. After connecting a robot to the operator interface and selecting it, as described in section 6.1.1, click on the “Teleop” button in the Map Window.
 	[fig]
ii.	The panel will expand to reveal the tele-operation interface as shown in Figure 
	[fig]
iii. Left clicking with the tele-operation interface will send a thrust and heading command to the boat. The heading sent is related to the horizontal location that is clicked in the tele-operation interface. The middle one gives the straight line heading, the left most one gives the strongest left heading possible and similarly the right most one gives the strongest right heading possible. The thrust sent is related to the vertical location clicked in the tele-operation interface. Clicking on the bottom of the red vertical lines will send 0% thust to the boat and clicking on top commands 100% thrust.

iv.	You can similarly right click to send the same command continuously or until you click again at another location or until you exit the tele-operation mode.

----------------
Autonomous Mode
----------------

The Platypus Lutra 1,x series supports three operating modes: Waypoints, Paths and Areas. The boat automatically switches to autonomous mode when any of these three tasks are chosen. At any point during the execution of these tasks, the operator can stop the boat and cancel the task by clicking on the “Cancel” button in the Map Frame Window.

--------------------
Assigning Waypoints
--------------------

Waypoint based navigation is the lowest level of autonomy in the Platypus Lutra 1.x series boats.

1.	First select a boat by clicking on the marker displayed in the Map Frame Window.
	[fig]
2.	Click on the “Point” button in the bottom of the Map Frame Window.
	[fig]
3.	Click anywhere on the map to indicate where the robot should move to. It will immediately start moving to that location.
	[fig]
4.	To command the robot to stop at any time, press the “Cancel” button in the bottom on the Map Frame Window.
	[fig]
5.	If you assign a waypoint while the boat was doing something else, it will first go to the new location and then continue what it was doing before.

6.	Click on the “Auto” button in the bottom of the Map Frame Window, if you want the boat to stop navigating to the waypoint immediately and return to what it was doing before.

----------------
Assigning Paths
----------------

You can assign a path that the boat should follow by clicking a series of waypoints.

1.	First select a boat by clicking on the marker displayed in the Map Frame Window.
	[fig]
2.	Click on the “Path” button in the bottom of the Map Frame Window.
	[fig]
3.	Click a series of points of the map to indicate the path that you want the boat to follow. To assign the last waypoint on the path, double click. The boat will immediately start following the path. 
	[fig]
4.	To command the robot to stop at any time, press the “Cancel” button in the bottom on the Map Frame Window.
	[fig]
5.	If you assign a path while the boat was doing something else, it will first complete the entire path and then continue what it was doing before.

6.	Click on the “Auto” button in the bottom of the Map Frame Window, if you want the boat to stop navigating the path immediately and return to what it was doing before.

------------------
Assigning an Area
------------------

In many applications, it is required that the boat continuously patrols in an area. 

1.	First select a boat by clicking on the marker displayed in the Map Frame Window.
	[fig]
2.	Click on “Explore Area” in the left panel of the Mission Monitor Window.
	[fig]
3.	Then click the “Run” button in the left panel of the Mission Monitor Window.
	[fig]
4.	Now in the Operator Interaction Frame Window, click a series of points to indicate the vertices of the polygon that you want the boat to explore. Double click to assign the last point.
	[fig]
5.	Click on the “Clear” button in the bottom of the Mission Monitor Window to reassign another area.
	[fig]
6.	You can also abort the area assignment by clicking on the “Cancel” button in the bottom of the Mission Monitor Window.
	[fig]
7.	After the area has been defined, click on the “Done” button in the bottom on the Mission Monitor Window.
	[fig]
8.	Select the boats to use by clicking on the list that is displayed in the Operator Interaction Frame Window. 
	[fig]
9.	Then click on the “Accept” button in the bottom of the Operator Interaction Frame Window.
	[fig]
10.	If more then one boat is chosen, the area is subdivided into lawn mower paths that each boat should follow.
	[fig]
11.	The boats will then start executing their paths.

-------------------------
Deploying Multiple Boats
-------------------------
For the most part, the procedure for deploying multiple boats is same as a single boat. Follow the same instructions for connecting to a boat and operating them. The only additional step in deploying multiple boats is how to switch between different boats to operate once they are connected, this section will give you instructions on how to achieve this. 

An important step while connecting multiple boats is giving each boat a unique color. By clicking on the Color button in the connection panel, the operator can choose a color for the boat in the initialization step. Each boat should be distinguished by a unique color. The boat markers on the map in the Map Window should now show each boat marker in the respective color of the boat that was chosen. In order to select a boat, click on the boat marker. The operator can now send commands to this selected boat. Similarly you can switch between different boats in the same way.

------------------------
Visualizing Sensor Data
------------------------
The operator interface also has the capability to display a heat map of the sensor readings overlaid on top of the map.

1.	Click on the “Visualize Data Source” drop down menu in the bottom of the Map Window.
	[fig]
2.	Click on the appropriate sensor name to select it.
	[fig]
3.	When the map is zoomed in, the heat map will be visualized at a 10m x 10m resolution.
	[fig]
4.	When the map is zoomed out, a subset of the heat map data will be visualized with constant pixel circles so that you can zoom out far.
	[fig]