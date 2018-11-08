#!/bin/bash
export ENVNAME="egm-demo"

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
export PATH=$PATH:$PROJECT_BASE/software/build/bin
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$PROJECT_BASE/software/build/lib:$PROJECT_BASE/software/build/lib64:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$PROJECT_BASE/software/build/lib/pkgconfig:$PROJECT_BASE/software/build/lib64/pkgconfig:$PKG_CONFIG_PATH
export PYTHONPATH=$PYTHONPATH:$PROJECT_BASE/software/build/lib/python2.7/site-packages:$PROJECT_BASE/software/build/lib/python2.7/dist-packages

# enable some warnings by default
export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"

export ROSLAUNCH_SSH_UNKNOWN=1

if [ -f $PROJECT_BASE/catkin_ws/devel/setup.bash ]; then
  source $PROJECT_BASE/catkin_ws/devel/setup.bash
  echo "Setting $ENVNAME environment"
else
  source /opt/ros/kinetic/setup.bash
fi
export ROS_PACKAGE_PATH=$PROJECT_BASE/ros_ws/:$ROS_PACKAGE_PATH

echo "Memory available:"
df -h --output=avail /

alias gitsub='git submodule update --init --recursive'
alias gitpull='git -C $PROJECT_BASE pull'

alias rebash='source ~/.bashrc'
alias open='gnome-open'

alias pman='bot-procman-sheriff -l $PROJECT_BASE/software/config/procman.pmd'

alias roslocal='export ROS_MASTER_URI=http://localhost:11311'

alias catmake='cd $PROJECT_BASE/catkin_ws; catkin_make; cd -;'
alias catrun='cd $PROJECT_BASE/catkin_ws; catkin_make -DCMAKE_CXX_FLAGS=-Ofast; cd -;'
alias catsim='cd $PROJECT_BASE/catkin_ws; catkin_make -DCMAKE_CXX_FLAGS=-Ofast -DCATKIN_BLACKLIST_PACKAGES="apriltags;realsense-ros;abb-ros-catkin;irb120_moveit_config;vicon_bridge"; cd -;'
alias myip="python -c 'import socket; print([l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith(\"127.\")][:1], [[(s.connect((\"8.8.8.8\", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0])'"

alias ..='cd ..'
alias c='clear'
alias lg='ls | grep'
alias lpy='ls | grep .py'
alias lla='ls -alF'
alias ll='ls -ltrF'
alias llpy='ls -ltrF | grep .py'
alias llg='ls -ltrF | grep'
alias l.='ls -d .* --color=auto'
alias listalias="alias -p | cut -d= -f1 | cut -d' ' -f2"
alias calculator='bc -l'
alias date='date'
alias meminfo='free -m -l -t'
alias gpull='git pull'
alias gpush='git push'
alias gstatus='git status'
alias h='hisotry'
alias hg='history | grep'

. /usr/share/autojump/autojump.bash
