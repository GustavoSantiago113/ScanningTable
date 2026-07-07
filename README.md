<div align="center">

# 3D Reconstruction of Small Objects

**Non-official implementation and adaptation of the paper: [Automated Low-Cost Photogrammetric Acquisition of 3D Models from Small Form-Factor Artefacts](https://www.mdpi.com/2079-9292/8/12/1441), from Collins et al., 2019**

**Note:** This implementation was performed in a **Linux** environment, with GPU with CUDA acceleration.

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

The camera was positioned a little bit different than described in the paper. Instead of at 40 degrees to the table and at 25cm distant, as describred, the images were took framing the whole calibration pattern, at less than 40 degrees. The reason for this is that phones have different focals, meaning that the images from different phones will become different, even if you keep the same parameters. *When doing this on your phone, try to make the board occupy the maximum of the image as possible and at low angles*.

Just like in the paper, I took some sets of different angles of the object (a 32mm scale minitature painted by me). The sets are in the folder [images](images/). Some examples:

**<center>Set 1:**

<img src="images\set_1\stop_03_20260703_163620.jpg" width="300" height="300"></center>

**<center>Set 2:**

<img src="images\set_2\stop_03_20260703_163907.jpg" width="300" height="300"></center>

**<center>Set 3:**

<img src="images\set_3\stop_07_20260703_164243.jpg" width="300" height="300"></center>

# Step 4 - Camera Geometry Estimation

Implemented in [camera_geometry_estimation.ipynb](camera_geometry_estimation.ipynb), using
[pycolmap with cuda](https://github.com/colmap/pycolmap) for feature extraction, matching and bundle
adjustment. The pipeline follows the paper: a sequence of virtual "photographs" of the calibration
plate is rendered from known viewpoints (paper spec: 12 views, 30&deg; apart, 45&deg; elevation;
the notebook defaults to 24 views/15&deg; steps for denser real-photo coverage - see its parameters
cell to reproduce the literal paper spec), registered with COLMAP at their exact known
intrinsics/pose, and used as a fixed calibration model that the real photographs are localised
against via incremental bundle adjustment (`fix_existing_frames` + `constant_cameras` keep the
virtual poses/intrinsics fixed throughout; only the real cameras are refined). A final
retriangulation pass (using every registered camera, not just the virtual ones) then fills out the
sparse point cloud with the artefact's own geometry, not just the calibration plate, before the
virtual photographs are removed, leaving calibrated real cameras plus a sparse point cloud ready
for dense reconstruction.

One thing this replica needed that the paper's summary doesn't spell out: **real photos only
reliably register when close in azimuth to a virtual view.** Matching a clean, synthetic render
against a real photograph (different lighting, print quality, JPEG compression, lens blur) is much
harder than matching two real photos to each other - hence the denser 24-view default above.

Not every real photograph is guaranteed to register - the notebook reports how many did, and is
intentionally bounded to a fixed runtime (`MAX_RUNTIME_SECONDS`, `MAX_REG_TRIALS=1`) rather than
retrying indefinitely. Registration also includes plausibility filters (implausible
camera-to-plate distance, near-duplicate positions from COLMAP's linear solver failing on a
poorly-conditioned planar-target fit) that discard failed estimates rather than keep them. If you hit not getting real cameras, consider tuning `ABS_POSE_MIN_NUM_INLIERS` /
`N_VIRTUAL_VIEWS` for the affected set, or re-running with a higher `MAX_REG_TRIALS`.

# Step 5 - Dense Point Cloud Reconstruction

Implemented in [dense_reconstruction.ipynb](dense_reconstruction.ipynb). The paper uses CMVS/PMVS
(Furukawa's clustering-views-for-multi-view-stereo toolchain) for this step. That toolchain has no
pip package, is unmaintained, and COLMAP's own docs describe it as superseded by COLMAP's own dense
pipeline - **PatchMatchStereo** (per-image depth/normal maps) followed by **StereoFusion** (merging
those into one point cloud) - which this notebook uses instead, via the same `pycolmap` library
already used for calibration.

The pipeline: load Step 4's calibrated real cameras and reuse the exact downscaled photographs it
already calibrated against (`outputs/<set>/work`, no separate re-downscale needed) &rarr; undistort
into a COLMAP dense workspace &rarr; PatchMatchStereo for per-image depth/normal maps &rarr;
StereoFusion into one dense, coloured point cloud &rarr; restrict to a rough region of interest
around the artefact (not the paper's dedicated cropping stage, which comes later) &rarr; attempt to
downsample towards target point densities (points per mm&sup2; of the ROI's bounding-box surface
area, standing in for actual surface area until a mesh exists).

**On target density**: this step was scoped for 100 and 300 points/mm&sup2;. Validated on `set_1`
(35 photos, downscaled to 2000x1500 as Step 4 left them), the achieved density came out to
**~6 points/mm&sup2;** - one to two orders of magnitude below either target. The notebook keeps
both targets as explicit parameters and reports the honest gap (via
`dense_reconstruction.downsample_to_density`'s `reached_target=False` path) rather than silently
substituting a different number or fabricating points a camera never actually resolved. This is a
property of the input photos (phone camera, ~20-25cm working distance, downscaled for tractable
runtime) more than the algorithm - raising Step 4's `MAX_LONG_EDGE` (more pixels on the artefact per
photo) and/or taking more, closer photos would likely close some of that gap, at the cost of a much
longer `PatchMatchStereo` runtime (full 4000x3000 photos measured ~60-90s/image on the dev GPU,
vs. ~35 minutes total for all of `set_1` at 2000x1500). Even though the poits density were not as high as stated in the paper, the reconstruction level of details is impressive.

# Step 6 - Cropping

# Step 7 - Point Cloud Registration

# Step 8 - Merge and Pruning

# Step 9 - Surface Mesh Reconstruction

# Step 10 - Texturing
