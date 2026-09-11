#!/usr/bin/env bash
# Avvia tutto quello che serve alla challenge.
#
#   ./run_mission.bash            Gazebo simulation + mission
#   ./run_mission.bash real       real robot + mission
#   ./run_mission.bash sim build  re-building before start
#

set -e

MODE=${1:-sim}
BUILD=${2:-}

# radice del workspace: la cartella che contiene src/
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [ "$WS" != "/" ] && [ ! -d "$WS/src" ]; do
    WS=$(dirname "$WS")
done
if [ ! -d "$WS/src" ]; then
    echo "Workspace non trovato (nessuna cartella src/ risalendo da qui)"
    exit 1
fi

export TURTLEBOT3_MODEL=burger

source /opt/ros/humble/setup.bash

if [ "$BUILD" = "build" ]; then
    echo ">>> colcon build in $WS"
    ( cd "$WS" && colcon build --symlink-install )
fi

source "$WS/install/setup.bash"

mkdir -p ~/challenge_output


trap 'kill 0' EXIT INT TERM

if [ "$MODE" = "real" ]; then
    echo ">>> robot reale: bringup + nav2 + missione"
    ros2 launch asr_summer_school bringup.launch.py use_sim_time:=false
else
    echo ">>> Gazebo"
    ros2 launch asr_summer_school project.launch.py &

    echo ">>> attendo Gazebo (15 s)"
    sleep 15

    echo ">>> bringup + nav2 + missione"
    ros2 launch asr_summer_school bringup_simulation.launch.py use_sim_time:=true &

    wait
fi
