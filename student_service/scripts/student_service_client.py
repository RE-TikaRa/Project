#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from student_service.srv import StudentQuery


def student_query_client(student_id):
    """客户端查询函数
    
    Args:
        student_id: 学号
        
    Returns:
        tuple: (姓名, 成绩) 或 ("", -1.0) 表示查询失败
    """
    try:
        student_query = rospy.ServiceProxy('student_query', StudentQuery)
        resp = student_query(student_id)
        return resp.name, resp.score
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)
        return "", -1.0


def print_student_info(student_id, name, score):
    """格式化打印学生信息"""
    print("\n" + "=" * 50)
    print("查询结果:")
    print(f"  学号: {student_id}")
    print(f"  姓名: {name}")
    print(f"  成绩: {score:.1f}")
    print("=" * 50)


if __name__ == "__main__":
    rospy.init_node('student_query_client')
    
    # 等待服务可用
    rospy.loginfo("等待服务 'student_query' 可用...")
    try:
        rospy.wait_for_service('student_query', timeout=10.0)
        rospy.loginfo("服务已连接")
    except rospy.ROSException:
        rospy.logerr("等待服务超时，请确保服务端已启动")
        print("\n错误：无法连接到学生查询服务")
        print("请先运行服务端: rosrun student_service student_service_server.py")
        sys.exit(1)
    
    # 打印欢迎信息
    print("\n" + "=" * 50)
    print("欢迎使用学生信息查询系统")
    print("=" * 50)
    
    while not rospy.is_shutdown():
        try:
            # 获取用户输入
            user_input = input("\n请输入学号 (输入 'q' 或 'quit' 退出): ").strip()
            
            # 检查退出条件
            if user_input.lower() in ['q', 'quit', 'exit']:
                rospy.loginfo("退出查询程序")
                break
            
            # 检查空输入
            if not user_input:
                print("输入不能为空，请重新输入")
                continue
            
            # 尝试转换为整数
            try:
                student_id = int(user_input)
            except ValueError:
                rospy.logwarn("无效的学号: '%s'，请输入数字", user_input)
                print(f"错误：'{user_input}' 不是有效的学号，请输入整数")
                continue
            
            # 验证学号范围
            if student_id <= 0:
                print("错误：学号必须是正整数")
                continue
            
            # 执行查询
            name, score = student_query_client(student_id)
            
            if score >= 0:
                rospy.loginfo("查询结果: 学号 %d, 姓名 %s, 成绩 %.1f", 
                            student_id, name, score)
                print_student_info(student_id, name, score)
            else:
                rospy.logwarn("未找到学号 %d 的学生", student_id)
                print(f"\n未找到学号 {student_id} 的学生记录")

            # 询问是否继续
            while True:
                next_step = input("\n是否继续查询？(y 继续 / n 退出): ").strip().lower()
                if next_step in ['y', 'yes', '']:
                    # 继续下一次查询（直接再次输入学号）
                    break
                elif next_step in ['n', 'no', 'q', 'quit', 'exit']:
                    rospy.loginfo("退出查询程序")
                    raise KeyboardInterrupt()
                else:
                    print("请输入 y 或 n（或 q/quit/exit 退出）")
                
        except KeyboardInterrupt:
            rospy.loginfo("\n检测到 Ctrl+C，退出查询程序")
            break
        except EOFError:
            rospy.loginfo("\n检测到输入结束，退出查询程序")
            break
        except Exception as e:
            rospy.logerr("发生错误: %s", e)
            print(f"错误：{e}")
            continue
    
    rospy.loginfo("查询程序已结束")
