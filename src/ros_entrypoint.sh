# This set of commands set user ID the same as the user ID of the host machine so ROS Nodes running in containers can communicate to Nodes on host machine
set -e
id -u ros &>/dev/null || adduser --quiet --disabled-password --gecos '' --uid ${UID:=1000} --uid ${GID:=1000} ros
exec "$@"