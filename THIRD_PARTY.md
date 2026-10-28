# Third-party notices

The ConPR++ player code is MIT licensed; see LICENSE. Dataset licensing is separate.

Only Livox message definitions are bundled. They do not include a hardware driver or SDK.
- ROS 1: `ros1/livox_ros_driver/msg`, copied from Livox-SDK/livox_ros_driver,
  commit `880c46a91aaa602dbecf20e204da4751747b3826`.
- ROS 2: `ros2/livox_interfaces/msg`, the CustomMsg/CustomPoint field definitions
  used by the existing ConPR++ player, derived from Livox's definitions.
- The upstream license is retained in each message package's LICENSE.

The ROS 1 compatibility package deliberately has the standard `livox_ros_driver`
name so existing consumers see the expected ROS datatype and MD5. If your workspace
already contains the real driver package, use that package instead of adding a
second package of the same name. ROS 2 publishes `livox_interfaces/msg/CustomMsg`,
not `livox_ros_driver2/msg/CustomMsg`; matching fields alone do not make those
package-qualified ROS 2 types interchangeable. PointCloud2 is the default output.

PyQt5, ROS, NumPy, Pillow, and other dependencies are installed separately under
 their respective licenses; no dependency binaries are shipped in the source bundle.
