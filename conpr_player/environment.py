"""Never silently replace the user's sourced ROS distribution."""
import os
ROS_DISTRO=os.environ.get('ROS_DISTRO','')
ROS_VERSION=os.environ.get('ROS_VERSION','')
if (ROS_DISTRO,ROS_VERSION) not in (('noetic','1'),('humble','2'),('jazzy','2')):
    raise RuntimeError('Source exactly one supported ROS environment first: Noetic, Humble or Jazzy. '
                       'Got ROS_DISTRO=%r ROS_VERSION=%r' % (ROS_DISTRO,ROS_VERSION))
ROS1=ROS_VERSION=='1'
DISPLAY_NAME='ROS %s %s' % (ROS_VERSION,ROS_DISTRO.capitalize())
