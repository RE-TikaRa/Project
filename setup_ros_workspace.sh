#!/bin/bash

set -e  # 遇到错误时退出脚本

echo "开始设置ROS工作空间..."

# 1. 在主文件夹下创建Tika_ws/src目录
echo "步骤1: 创建工作空间目录 ~/Tika_ws/src"
mkdir -p ~/Tika_ws/src
echo "✓ 目录创建完成"

# 2. 在src下运行catkin_init_workspace
echo "步骤2: 初始化catkin工作空间"
cd ~/Tika_ws/src
catkin_init_workspace
echo "✓ catkin工作空间初始化完成"

# 3. 回到Tika_ws目录
echo "步骤3: 返回到Tika_ws目录"
cd ~/Tika_ws
echo "✓ 已切换到工作空间根目录"

# 4. 执行catkin_make with specific Python executable
echo "步骤4: 执行catkin_make编译"
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
echo "✓ catkin_make编译完成"

# 5. 进入src目录
echo "步骤5: 进入src目录"
cd src
echo "✓ 已进入src目录"

# 6. 克隆项目仓库的main分支
echo "步骤6: 克隆GitHub项目"
if [ -d "Project" ]; then
    echo "警告: Project目录已存在，正在删除..."
    rm -rf Project
fi
git clone -b main https://github.com/RE-TikaRa/Project.git
echo "✓ 项目克隆完成"

# 7. 给克隆下来的所有文件添加执行权限
echo "步骤7: 为所有文件添加执行权限"
find Project -type f -exec chmod +x {} \;
echo "✓ 执行权限设置完成"

# 8. 创建ROS包student
echo "步骤8: 创建catkin包student"
catkin_create_pkg student rospy std_msgs roscpp message_generation
echo "✓ catkin包student创建完成"

# 9. 复制student_service下的指定目录到student包
echo "步骤9: 复制student_service下的指定目录到student包"
if [ -d "Project/student_service" ]; then
    # 复制data目录
    if [ -d "Project/student_service/data" ]; then
        cp -r Project/student_service/data student/
        echo "  ✓ data目录已复制"
    else
        echo "  ⚠ data目录不存在，跳过"
    fi
    
    # 复制scripts目录
    if [ -d "Project/student_service/scripts" ]; then
        cp -r Project/student_service/scripts student/
        echo "  ✓ scripts目录已复制"
    else
        echo "  ⚠ scripts目录不存在，跳过"
    fi
    
    # 复制srv目录
    if [ -d "Project/student_service/srv" ]; then
        cp -r Project/student_service/srv student/
        echo "  ✓ srv目录已复制"
    else
        echo "  ⚠ srv目录不存在，跳过"
    fi
    
    echo "✓ 指定目录复制完成"
else
    echo "警告: Project/student_service目录不存在，跳过复制步骤"
fi


