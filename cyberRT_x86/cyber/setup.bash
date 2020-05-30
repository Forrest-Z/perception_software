export CYBER_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")/../" && pwd)
binary_path="${CYBER_PATH}/bin"
library_path="${CYBER_PATH}/lib"
cyber_tool_path="${binary_path}/cyber/tools"
recorder_path="${cyber_tool_path}/cyber_recorder"
monitor_path="${cyber_tool_path}/cyber_monitor"
PYTHON_LD_PATH="${library_path}/cyber/cyber_py3"
launch_path="${CYBER_PATH}/tools/cyber_launch"
channel_path="${CYBER_PATH}/tools/cyber_channel"
node_path="${CYBER_PATH}/tools/cyber_node"
service_path="${CYBER_PATH}/tools/cyber_service"
#qt_path=/usr/local/Qt5.5.1/5.5/gcc_64

export LD_LIBRARY_PATH=${library_path}:$LD_LIBRARY_PATH
export PATH=${binary_path}:${recorder_path}:${monitor_path}:${launch_path}:${channel_path}:${node_path}:${service_path}:$PATH
export PYTHONPATH=${PYTHON_LD_PATH}:${library_path}/python:$PYTHONPATH

export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1

export GLOG_log_dir=/apollo/data/log
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

export sysmo_start=0

# for DEBUG log
#export GLOG_minloglevel=-1
#export GLOG_v=4

source ${CYBER_PATH}/bin/cyber/tools/cyber_tools_auto_complete.bash
