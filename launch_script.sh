#!/usr/bin/env bash
set -e

GPU="device=1"
echo "GPU: $GPU"

IMAGES_VERSION="1.0.0"
echo "IMAGES_VERSION: $IMAGES_VERSION"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "PROJECT_ROOT: $PROJECT_ROOT"

DATA_ROOT="$(dirname "$PROJECT_ROOT")"
echo "DATA_ROOT: $DATA_ROOT"



if [ ! -d $PROJECT_ROOT ]; then
  echo "ERROR: $PROJECT_ROOT 不存在"
  exit 1
fi


if [ ! -d $DATA_ROOT ]; then
  echo "ERROR: $DATA_ROOT 不存在"
  exit 1
fi

# 检查 .env
if [ ! -f "$PROJECT_ROOT/.env" ]; then
  echo "ERROR: 不存在 .env 文件"
  exit 1
fi


read -rp "是否確認啟動？[y/N] " reply
case "$reply" in
  [Yy]*) echo "Starting Custom Isaac Sim container...";;
  *) echo "已取消。"; exit 0;;
esac



# echo "Starting Custom Isaac Sim container..."


docker run --name isaac_sim_lab_container \
    --gpus $GPU \
    -it --rm \
    --network=host \
    --env-file "$PROJECT_ROOT/.env" \
    isaac_sim_lab_ros2_nav2:$IMAGES_VERSION

