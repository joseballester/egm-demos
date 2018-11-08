#!/bin/bash
export ENVNAME="egm-demo"
export USEREMAIL="jose@ballester.me"
export USERNAME="Jose Ballester"

thisFile=$_
if [ $BASH ]
then
  # may be a relative or absolute path
  thisFile=${BASH_SOURCE[0]}
fi

# use cd and pwd to get an absolute path
configParentDir="$(cd "$(dirname "$thisFile")/.." && pwd)"

# different cases for software/config or software/build/config
case "$(basename $configParentDir)" in
  "software") export PROJECT_BASE=$(dirname $configParentDir);;
  "build") export PROJECT_BASE=$(dirname $(dirname $configParentDir));;
  *) echo "Warning: $ENVNAME environment file is stored in unrecognized location: $thisFile";;
esac

export PROJECT_DATA_BASE=$PROJECT_BASE/data

# enable some warnings by default
export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"

export ROSLAUNCH_SSH_UNKNOWN=1

echo "Memory available:"
df -h --output=avail /
cd $PROJECT_BASE; git config user.email "$USEREMAIL"; git config user.name "$USERNAME"; cd -;

if [ -f $PROJECT_BASE/catkin_ws/devel/setup.bash ]; then
  source $PROJECT_BASE/catkin_ws/devel/setup.bash
  echo "Setting $ENVNAME environment"
else
  source /opt/ros/kinetic/setup.bash
fi
export ROS_PACKAGE_PATH=$PROJECT_BASE/ros_ws/:$ROS_PACKAGE_PATH

alias rebash='source ~/.bashrc'
alias open='gnome-open'
alias pman='bot-procman-sheriff -l $PROJECT_BASE/software/config/procman.pmd'
alias catmake='cd $PROJECT_BASE/catkin_ws; catkin_make; cd -;'

. /usr/share/autojump/autojump.bash
