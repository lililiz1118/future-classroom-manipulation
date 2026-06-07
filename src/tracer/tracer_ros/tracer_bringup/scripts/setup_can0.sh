#!/bin/bash
# 自动配置 CAN0 波特率

# 检查 can0 是否已配置正确
if ip link show can0 2>/dev/null | grep -q "bitrate 500000"; then
    echo "[setup_can0] can0 已配置 500000，跳过"
    exit 0
fi

echo "[setup_can0] 正在设置 can0 bitrate 500000..."

# 先 down 掉已有的 can0（如果存在），否则无法修改参数
echo "123" | sudo -S ip link set can0 down 2>/dev/null

# 设置波特率并 up
echo "123" | sudo -S ip link set can0 up type can bitrate 500000
ret=$?

if [ $ret -eq 0 ]; then
    echo "[setup_can0] can0 配置成功"
    sleep 0.5
else
    echo "[setup_can0] can0 配置失败！exit code: $ret"
    exit 1
fi
