#!/bin/bash
# ============================================================
# stop_all.sh — 一键停止所有 myrobot 相关进程
#
# 用法：Ctrl+C 关闭 launch 后，如果还有残留进程，运行：
#     ./stop_all.sh
#
# 说明：pkill -f 匹配完整命令行；模式中的 [x] 方括号技巧
#       确保脚本/调用者自身不会被误杀
# ============================================================

echo "正在停止 ROS 进程..."

# 停止 launch 本身
pkill -9 -f "ros2 launch myrob[o]t" 2>/dev/null

# 停止所有子进程（节点/控制器/规划/可视化）
pkill -9 -f "ros2_control_nod[e]" 2>/dev/null
pkill -9 -f "robot_state_publishe[r]" 2>/dev/null
pkill -9 -f "move_grou[p]" 2>/dev/null
pkill -9 -f "commander_templat[e]" 2>/dev/null
pkill -9 -f "test_movei[t]" 2>/dev/null
pkill -9 -f "spawne[r]" 2>/dev/null
pkill -9 -f "rviz[2]" 2>/dev/null

sleep 1

# 复查残留
LEFT=$(ps -e | grep -E "ros2_control|robot_state|move_group|commander|spawner|rviz" | grep -v grep)
if [ -z "$LEFT" ]; then
    echo "✅ 所有进程已清理干净"
else
    echo "⚠️ 仍有残留进程："
    echo "$LEFT"
fi
