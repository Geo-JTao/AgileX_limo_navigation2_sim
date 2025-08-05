#!/bin/bash

# 检查是否提供了关键词参数
if [ $# -eq 0 ]; then
    echo "用法: $0 <进程关键词>"
    echo "示例: $0 nav"
    exit 1
fi

keyword=$1

# 查找包含关键词的进程ID，排除grep自身
pids=$(ps -aux | grep "$keyword" | grep -v grep | awk '{print $2}')

if [ -z "$pids" ]; then
    echo "没有找到包含 '$keyword' 的进程"
    exit 0
fi

echo "找到以下进程ID: $pids"
echo "正在杀死这些进程..."

# 杀死找到的进程
sudo kill -9 $pids

# 检查是否成功杀死
if [ $? -eq 0 ]; then
    echo "进程已成功终止"
else
    echo "杀死进程时出现错误"
    exit 1
fi
