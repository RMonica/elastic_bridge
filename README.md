Elastic Bridge
==============

The `elastic_bridge` package integrates ElasticFusion by mp3guy into ROS (Robot Operating System).

<https://github.com/mp3guy/ElasticFusion>

The package is non-interactive. It only depends on the ElasticFusion Core module, instead of the GUI module. Moreover, it can be used with any RGB-D sensor with a ROS driver, as it does not depend on OpenNI.

Dependencies (in addition to ElasticFusion dependencies):

- ROS (Robot Operating System)
- PCL (Point Cloud Library)
- `init_fake_opengl_context`: badly-named package to initialize a windowless OpenGL context (<https://github.com/RMonica/init_fake_opengl_context>)

The package was tested on ROS Kinetic (Ubuntu 16.04) and ROS Melodic (Ubuntu 18.04).

An optional patch may be applied to the ElasticFusion source code. The patch assigns an unique identifier (GUID) to each surfel, so that single surfels can be tracked during the 3D reconstruction process. For example, you may keep track of extra properties for each surfel in an external ROS node. See "GUID patch" below for more information.

### Installation

- Download this repository, `elastic_bridge`, into your ROS workspace
- Download `init_fake_opengl_context` into your ROS workspace, from <https://github.com/RMonica/init_fake_opengl_context>.
- Download ElasticFusion from <https://github.com/mp3guy/ElasticFusion> into `[...]elastic_bridge/deps/ElasticFusion`.
- Download Pangolin from <https://github.com/stevenlovegrove/Pangolin> into `[...]elastic_bridge/deps/ElasticFusion/deps/Pangolin`.
- Compile Pangolin:  
```
cd [...]elastic_bridge/deps/ElasticFusion/deps/Pangolin
mkdir build
cd build  
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```
- **Optional**: apply the GUID patch
```
cd [...]elastic_bridge/deps/ElasticFusion
patch -p1 -i ../../patches/guid.patch
```
- Compile ElasticFusion Core module
```
cd [...]elastic_bridge/deps/ElasticFusion/Core
mkdir build
cd build
cmake ../src
make
```
- Compile the package (`catkin build` is recommended instead of `catkin_make`):
```
catkin build elastic_bridge
```

### Usage

An example launch file for Kinect v1 is provided in the `launch` folder.
The main node is `elastic_node`.

Set parameters `TOPIC_IMAGE_COLOR` to the color image topic, `TOPIC_IMAGE_DEPTH` to the depth image topic, and `TOPIC_CAMERA_INFO` to the camera info topic. The node waits for the first camera info before creating the ElasticFusion instance, as ElasticFusion cannot be initialized before image width and height are known.

By default, the node starts in suspended state. Set `AUTOSTART` to `true` if the node should start 3D reconstruction immediately. Otherwise, the node can be un-suspended by sending a `std_msgs/Empty` message to `/elastic_scan_start`. The node can be suspended again by sending a `std_msgs/Empty` to `/elastic_scan_end`.

During execution, the node publishes the current 3D reconstruction image in `/elastic_current_view`. Current sensor pose, as estimated by egomotion tracking, is published to TF frame `/camera_frame`, with reference `/first_frame`, corresponding to the first camera pose.

It is possible to provide an external tracking source by setting parameter `TF_POSE_ALWAYS` to `true`. The transformation is read by default between the TF frames `/world` and `/robot`.

The surfel-based 3D reconstruction can be downloaded as a point cloud of `pcl::PointSurfel`. For this, the action `/save_pcl` may be called. An example is provided in the node `save_pcl`, compiled from `src/save_pcl.cpp`.

### Parameters

- `TOPIC_IMAGE_COLOR` (string): color image topic.
- `TOPIC_IMAGE_DEPTH` (string): depth image topic.
- `TOPIC_CAMERA_INFO` (string): camera info topic.
- `WORLD_FRAME` (string): published TF reference frame (default: `/first_frame`)
- `CAMERA_FRAME` (string): published TF camera frame (default: `/camera_frame`)
- `AUTOSTART` (bool): if `false`, the node is in suspended state at startup (default: `false`)
- `DISPLAY_NAME` (string): X display for the OpenGL context (default: auto-detect)
- `BLACK_TO_NAN` (bool): some cameras set invalid RGB pixels as pure black, if this option is set such pixels are ignored even if they have valid depth (default: `false`)
- `PERIODIC_WORLD_PUBLICATION` (int): every `PERIODIC_WORLD_PUBLICATION` frames, the surfel cloud is published as specified by `TOPIC_PERIODIC_WORLD_PUB` (default 0: disabled)
- `TOPIC_PERIODIC_WORLD_PUB` (string): topic in which the surfel cloud should be published as `sensor_msgs/PointCloud2` (default: `/elastic_world_pub`)
- `TOPIC_CURRENT_VIEW` (string): at each frame, a RGB image showing the view as predicted from the current sensor pose is published to this topic (default: `/elastic_current_view`)
- `TOPIC_FRAME_STATE` (string): at each frame, a message of type `msg/FrameState.msg` is published, which contains additional information for each pixel of the predicted view, such as position, normal, etc. (default: `/elastic_frame_state_stable`)
- `TF_POSE_ALWAYS` (bool): if true, sensor pose is read from TF (default: `false`)
- `TF_POSE_FIRST` (bool): if true, the sensor pose is read from TF only for the first sensor frame (default: `false`)
- `TF_INPUT_WORLD_FRAME` (string): set TF reference frame for external sensor pose tracking (default: `/world`)
- `TF_INPUT_CAMERA_FRAME` (string): set TF camera frame for external sensor pose tracking (default: `/robot`)
- `TOPIC_SCAN_READY` (string): topic to un-suspend the node (default: `/elastic_scan_start`)
- `TOPIC_SCAN_FINISH` (string): topic to suspend again the node (default: `/elastic_scan_end`)
- `SAVE_PCL_ACTION` (string): action name to retrieve the surfel cloud (default: `/save_pcl`)

### GUID patch

Surfels can be reordered by ElasticFusion during execution. With the GUID patch, the node also tracks each surfel by assigning an unique ID to each of them. Two kind of UIDs are tracked:

- A LUID (32-bit integer) identifies a surfel as long as it exists, but it can be re-assigned to new surfels if it is destroyed
- A GUID (64-bit integer) identifies a surfel and it is never reused even if the surfel is destroyed

LUIDs and GUIDs are returned in the frame state, one for each pixel in the image, in the topic as defined by parameter `TOPIC_FRAME_STATE`. The message type `FrameState.msg` contains, among other things:

- `guid`: array of GUIDs, one for each pixel
- `luid`: array of LUIDs, one for each pixel
- `luid_removed`: LUIDs of the surfels destroyed in this frame, which are no longer valid and may be reused in the next frame.
- `max_luid`: maximum currently existing LUID

GUIDs and LUIDs are also returned by the `save_pcl` action, one for each surfel (see action definition `action/downloadPointcloud2.action`.

### Acknowledgments

The original `elastic_bridge` package was written by Andrea Pagani during his bachelor thesis in Computer, Electronics and Telecommunication Engineering at University of Parma, Italy, in 2017-2018.
