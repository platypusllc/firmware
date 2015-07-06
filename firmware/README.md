# Platypus Controller Firmware #

This firmware is used by the Platypus Controller to interface the hardware on the vehicle with the Platypus Server, which runs in Android.

## Installation Setup ##

1. Download the latest version of the [Arduino IDE](1).
2. Navigate to **Tools > Board: "..." > Boards Manager...**
3. Install the following board support package:
  - **Arduino SAM Boards (32-bits ARM Cortex-M3** by **Arduino**
4. Navigate to **Sketch > Include Library > Manage Libraries...**
5. Install the following libraries:
  - **Scheduler** by **Arduino** [(more info)][2]
  - **USBHost** by **Arduino** [(more info)][3]

## Building/Uploading ##

1. Plug a USB cable from your PC into the **Programming USB Port** labeled on the DUE.
2. Select **Tools > Board: "..." > Arduino Due (Programming Port)**
3. Select **Tools > Port > "..." (Arduino Due (Programming Port))**
4. Press the **Upload** button (right-arrow icon)

[1]: https://www.arduino.cc/en/Main/Software
[2]: https://www.arduino.cc/en/Reference/Scheduler
[3]: https://www.arduino.cc/en/Reference/USBHost
