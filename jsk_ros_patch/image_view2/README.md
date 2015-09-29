image_view2
==========
`image_view2` is an extended [image_view](http://ros.org/wiki/image_view).

Features
* Draw markers on image view
* Interact with user direction

image_view2/ImageMarker
-----------------------
image_view2/ImageMarker is a message to draw on image_view2 canvas.

Definition is:
```
byte CIRCLE=0
byte LINE_STRIP=1
byte LINE_LIST=2
byte POLYGON=3
byte POINTS=4
byte FRAMES=5
byte TEXT=6
byte LINE_STRIP3D=7
byte LINE_LIST3D=8
byte POLYGON3D=9
byte POINTS3D=10
byte TEXT3D=11
byte CIRCLE3D=12
byte ADD=0
byte REMOVE=1
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string ns
int32 id
int32 type
int32 action
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
geometry_msgs/PointStamped position3D
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Point point
    float64 x
    float64 y
    float64 z
geometry_msgs/PoseStamped pose
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
float32 scale
float32 width
std_msgs/ColorRGBA outline_color
  float32 r
  float32 g
  float32 b
  float32 a
byte filled
std_msgs/ColorRGBA fill_color
  float32 r
  float32 g
  float32 b
  float32 a
duration lifetime
byte arc
float32 angle
geometry_msgs/Point[] points
  float64 x
  float64 y
  float64 z
image_view2/PointArrayStamped points3D
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Point[] points
    float64 x
    float64 y
    float64 z
std_msgs/ColorRGBA[] outline_colors
  float32 r
  float32 g
  float32 b
  float32 a
string[] frames
string text
bool left_up_origin
bool ratio_scale
```