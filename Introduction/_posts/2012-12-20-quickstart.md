---
layout: default
title: Quickstart Guide
---

# Quickstart Guide

## Power Boat
1. Plug the Battery Power Connector into the ESC Power Connector.
1. Listen for beeps from the motor (2 slow beeps, then 3 fast beeps), you should also hear the fan on the ESC running.
1. Partially close Electronics Compartment (close it as far as it will go, there should be about a 2” opening at the front.)

## Set up Phone
1. Remove phone enclosure from boat.
1. Put phone into enclosure and turn on.
1. Connect to local WiFi network
    1. Go to application list
    1. Go to settings
    1. WiFi - this will list available networks
    1. Select desired network
 
1. Open Amarino 2.0 App
    1. Go to application list
    1. Press connect to appropriate Arduino Bluetooth device (for Shady Side Academy prototype boat, this will always be AIRBOAT HW)   
    1. Red connect indicator should turn to green, connect button should turn to disconnect
    1. If connection fails, restart boat and try again
1. Go back to Application List
1. Open Airboat Setup Application, 
    1. Delete Airboat Bluetooth MAC Address, then type 00
    1. This will bring up the MAC Address of the Bluetooth Device that the phone is currently connected to.  Click that address.
    1. Then click the "Off" button (this actually turns it on, and changes the status to "On")
    1. If you have properly done all of the steps so far, then the IP address of the phone should appear under the UDP Address label (below the "On" button).
    1. Write down IP address, including the port # (the port is always 11411)
       **Note:** if you only ever use the same phone and the same boat, you can skip this step.
1. Verify Connections
    1. Still in Airboat Setup App
    1. At this point, the phone is connected both to the boat electronics (via Bluetooth) and to the local WiFi network.
    1. To verify the connections, press the Debug button
    1. Gently nudge Thrust slider up, Airboat fan should turn on.  Move it back to zero
    1. Gently slide Rudder slider back and forth, the Airboat turret should move back and forth accordingly.
    1. If this doesn't work, go back to beginning and try these instructions again.  If it still doesn't work, call us!
1. Verify GPS signal lock
    1. Open Google Maps app
    1. Click current location icon (top right corner of screen)
    1. If it shows you position, it has a lock.  Otherwise it will say "waiting for location" or "location cannot be obtained".  If you get one of these messages, wait a few seconds then click current location icon again until you succeed.
1. Close Phone Enclosure and attach to Enclosure to boat. The Enclosure just slides onto the mounting bracket and is secure by Velcro.

## Set up Controller Laptop
1. Connect Laptop to same WiFi Network as the boat is connected to.
1. Launch Airboat App on laptop (for now you have to run from NetBeans):
    1. Open NetBeans
    1. Open crw App
    1. Open edu.cmu.ri.airboat.floodtest
    1. Right click Operator Console, click run
1. This bring up two panels: the Connection Panel and the Operator Console.
1. In Connection Panel
    1. enter IP address and port # into the Server field (e.g. 128.237.121.126:11411).
    1. Assign a color to the boat: click on the Color button, select a color.
    1. Press Create button in the Physical section.
1. In Operator Console
    1. Zoom out on the map until you see the whole Earth
    1. You should see the boat on the Earth, represented by a triangle of the color selected above.
    1. Zoom in on the boat triangle, this should be your actual position.

Now you're ready to go!

{% include navbar.ext %}
