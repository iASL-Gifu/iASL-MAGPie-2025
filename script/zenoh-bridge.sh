#!/bin/bash

# --- コマンドライン引数の処理 ---
ROLE=${1:-jetson}        # jetson or host
NETWORK=${2:-hotspot}    # hotspot or wifi

# --- 接続先設定 ---
if [ "$NETWORK" = "wifi" ]; then
  CONNECT_IP="tcp/192.168.11.82:7447"
  LISTEN_IP="tcp/192.168.11.135:7447"
else
  # デフォルトはhotspot
  CONNECT_IP="tcp/10.42.0.25:7447"
  LISTEN_IP="tcp/10.42.0.1:7447"
fi

# hostならconnectとlistenを入れ替える
if [ "$ROLE" = "host" ]; then
  TMP="$CONNECT_IP"
  CONNECT_IP="$LISTEN_IP"
  LISTEN_IP="$TMP"
fi

# --- allowリストの設定（複数トピック） ---
PUB=( "/sensors/imu/raw" "/scan" )
SUB=( "/joy" "/cmd_vel" )

if [ "$ROLE" = "host" ]; then
  TMP=("${PUB[@]}")
  PUB=("${SUB[@]}")
  SUB=("${TMP[@]}")
fi

# --- 設定ファイルの生成 ---
CONFIG_FILE=$(mktemp /tmp/zenoh_config.XXXXXX.json5)

cat <<EOF > "$CONFIG_FILE"
{
  plugins: {
    ros2dds: {
      allow: {
        publishers: [
$(for topic in "${PUB[@]}"; do echo "          \"$topic\","; done)
        ],
        subscribers: [
$(for topic in "${SUB[@]}"; do echo "          \"$topic\","; done)
        ],
        service_servers: [],
        service_clients: [],
        action_servers: [],
        action_clients: []
      }
    }
  },
  mode: "peer",
  connect: {
    endpoints: ["$CONNECT_IP"]
  },
  listen: {
    endpoints: ["$LISTEN_IP"]
  }
}
EOF

# --- 実行 ---
EXECUTABLE="zenoh-bridge-ros2dds"

if command -v "$EXECUTABLE" >/dev/null 2>&1; then
  echo "[INFO] Starting $EXECUTABLE with role=$ROLE, network=$NETWORK"
  $EXECUTABLE --config "$CONFIG_FILE"
else
  echo "Error: $EXECUTABLE not found"
  rm -f "$CONFIG_FILE"
  exit 1
fi

# --- 終了時に設定ファイル削除 ---
trap "rm -f $CONFIG_FILE" EXIT
