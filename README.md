<div align="center">

# 3D Reconstruction of Small Objects

**Non-official implementation and adaptation of the paper: [Automated Low-Cost Photogrammetric Acquisition of 3D Models from Small Form-Factor Artefacts](https://www.mdpi.com/2079-9292/8/12/1441), from Collins et al., 2019**

</div>

---

- [3D Reconstruction of Small Objects](#3d-reconstruction-of-small-objects)
- [Step 1 - Image Acquisiton hardware](#step-1---image-acquisiton-hardware)
- [Step 2 - Acquisition Software](#step-2---acquisition-software)
- [Step 3 - Taking Images](#step-3---taking-images)
- [Step 4 - Camera Geometry Estimation](#step-4---camera-geometry-estimation)
- [Step 5 - Dense Point Cloud Reconstruction](#step-5---dense-point-cloud-reconstruction)
- [Step 6 - Cropping](#step-6---cropping)
- [Step 7 - Point Cloud Registration](#step-7---point-cloud-registration)
- [Step 8 - Merge and Pruning](#step-8---merge-and-pruning)
- [Step 9 - Surface Mesh Reconstruction](#step-9---surface-mesh-reconstruction)
- [Step 10 - Texturing](#step-10---texturing)


---

# Step 1 - Image Acquisiton hardware

For the table, I 3D modeled and printed a structure for it. It has a base and a top plate. You can find the models in the folder [3DFiles](3DFiles/). You can print it in the way you prefer, but I recommend using PLA, 3 layers of wall width, 20% fill, and 0.2mm layer height. To aid in the table movement, I added **3 2809 bearings.** To fix the motor in place, I used 2 3mm bolts.

The pseudo-random calibration pattern was made using a code in Python, available at the file [generate_pattern.py](utils/generate_pattern.py). It generates the pattern of a 130x130 mm square with 52 squares at each side, and saves as an image.

For the hardware, I used a 28BYJ-48 Stepper Motor with a ULN2003AN DIP-16 Driver. To connect with the phone and control the motor, I am using a Wemos D1 Mini ESP8266 Board. To power it, I am using a 9V power supply and a L7805 voltage regulator, with some capacitors. To connect everything, I made a custom PCB. The bill of materials, schematics and gerber files are in the folder [PCB](PCB/).

<img src="media/IMG_20260521_174254.jpg" width="300" height="300">


For the lights, I followed the same method as described by the paper, with a desk lamp at 20 cm perpendicullarly appart from the rotating plate.

<img src="media/IMG_20260521_174245.jpg" width="300" height="300">


# Step 2 - Acquisition Software

The script to run the ESP8266 is available at [ESP8266.ino](ESP8266\ESP8266.ino). The number of rotations is fixed on 36 (10 degree interval). The ESP creates a WebServer, with a Wi-Fi name and a password. The app sends requests to the ESP, such as: `start`, `stop`, and `continue`. The ESP sends the command `waiting` to the app so it knows when to take the picture.

The [app](scanning_table_app) was made in *Flutter* and **tested on an Android device only**. It has a initial page, only allowing to move to the next page (taking pictures) when the cellphone is connected to the ESP. To start scanning the user first must to choose the folder where the images are going to be saved. After that, the user can click on `Scan` and the app + ESP does the rest automatically. If something goes wrong, the user can click on `stop`, and the operation will be stopped.

The camera settings in the app are following the paper description:
- Apperture of f/1.2;
- Shutter speed of 1/4s;
- ISO 200;

**Important reminder: to connect with the ESP, the mobile data must be turned off**

# Step 3 - Taking Images

# Step 4 - Camera Geometry Estimation

# Step 5 - Dense Point Cloud Reconstruction

# Step 6 - Cropping

# Step 7 - Point Cloud Registration

# Step 8 - Merge and Pruning

# Step 9 - Surface Mesh Reconstruction

# Step 10 - Texturing
