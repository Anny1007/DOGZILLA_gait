#!/usr/bin/env bash
# DOGZILLA S1 步態測試環境 (與 ../DOGZILLA/sim_env 共用 Docker 映像與 CHAMP 編譯結果)
# 用法:
#   ./run.sh sim [flat|slope|slope_5] [launch 參數...]   啟動模擬 (有 Gazebo 畫面)
#                                 例: ./run.sh sim flat lidar_mass:=0.17 lidar_z:=0.10
#   ./run.sh suite <名稱> [flat|slope] [launch 參數...]   跑整套測試 (無頭), 結果存 results/<名稱>/
#                                 平地: gait_eval (直走/原地轉/邊走邊轉) + combo_test (翻倒邊界) + push_test (被推)
#                                 斜坡: slope_test
#                                 例: ./run.sh suite baseline flat
#                                     ./run.sh suite lidar170 flat lidar_mass:=0.17 lidar_z:=0.10
#   ./run.sh shell                進到正在跑的模擬容器
#   ./run.sh stop                 停止模擬
#
# 注意: 容器用主機網路, 與主專案 (dogzilla-sim) 同時跑會互相干擾, 所以兩者只能擇一。
cd "$(dirname "$0")"
NAME=dogzilla-gait
COMBOS="0.1,0.4 0.15,0.5 0.2,0.0 0.2,0.2 0.2,0.5 0.2,1.0 0.2,-1.0"   # S1 步態最高 0.2 m/s (config/gait_s1.yaml)
running() { docker ps --format '{{.Names}}' | grep -qx "$1"; }

start() {   # start <world> [launch 參數...]
  local world=$1; shift
  running dogzilla-sim && { echo "主專案的模擬 (dogzilla-sim) 正在跑, 請先在 ../DOGZILLA/sim_env 執行 ./run.sh stop"; exit 1; }
  running $NAME && { echo "模擬已在執行 (./run.sh stop 先停止)"; exit 1; }
  xhost +local:docker >/dev/null 2>&1
  : > logs/sim.log
  docker compose run -d --rm --name $NAME gait bash -lc \
    "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 launch /launch/gait_test.launch.py world:=/worlds/$world.world $* 2>&1 | tee /logs/sim.log" >/dev/null
}

wait_ready() {
  for _ in $(seq 120); do
    grep -q "\[unpause\]" logs/sim.log 2>/dev/null && { sleep 3; grep "\[unpause\]" logs/sim.log | tail -1; return 0; }
    sleep 1
  done
  echo "啟動逾時, 見 logs/sim.log"; return 1
}

in_sim() { docker exec $NAME bash -lc "source /opt/ros/humble/setup.bash; $*"; }

case "$1" in
  sim)
    shift; world=${1:-flat}; [ $# -gt 0 ] && shift
    start $world "$@"
    echo "模擬啟動中 (場地 $world), 約 20 秒後可測試。日誌: logs/sim.log" ;;
  suite)
    label=${2:?請給測試名稱}; world=${3:-flat}; shift; shift; [ $# -gt 0 ] && shift
    out=results/$label; mkdir -p $out
    echo "# 場地 $world, 參數: $*  ($(date '+%F %T'))" > $out/config.txt
    if [[ $world == slope* ]]; then
      tests="slope_test.py"
    else
      tests="gait_eval.py combo_test.py push_test.py"
    fi
    for t in $tests; do
      docker rm -f $NAME >/dev/null 2>&1; sleep 2
      start $world "$@" && wait_ready || exit 1
      args=""; [ $t = combo_test.py ] && args=$COMBOS
      echo "== $t"
      in_sim python3 -u /tools/$t $args 2>&1 | grep -v Warning | tee $out/${t%.py}.txt
    done
    docker rm -f $NAME >/dev/null 2>&1
    echo "== 完成, 結果在 $out/" ;;
  shell)
    if running $NAME; then docker exec -it $NAME bash; else docker compose run --rm gait bash; fi ;;
  stop) docker rm -f $NAME >/dev/null 2>&1 && echo "已停止" ;;
  *) sed -n 2,14p "$0" ;;
esac
