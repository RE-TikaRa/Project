#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import os
import sys
import time
import threading
from student_service.srv import StudentQuery, StudentQueryResponse


students = {}
stats = {
    'total': 0,
    'hits': 0,
    'misses': 0,
    'startup_ts': time.time(),
    'seq': 0,
}
json_file_path = None
_last_mtime = 0.0

# 保护 students 的并发访问
students_lock = threading.Lock()


def _vlog(enabled, level, msg, *args):
    if not enabled:
        return
    if level == 'info':
        rospy.loginfo(msg, *args)
    elif level == 'warn':
        rospy.logwarn(msg, *args)
    else:
        rospy.logdebug(msg, *args)


def find_json_file():
    """查找JSON文件（默认 ../data/students.json，支持 ~json_file 覆盖；不再交互输入以避免 roslaunch 场景下卡住）。"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_path = os.path.normpath(os.path.join(script_dir, '..', 'data', 'students.json'))

    # 优先使用默认路径
    if os.path.exists(default_path):
        rospy.loginfo("找到JSON文件: %s", default_path)
        return default_path

    # 支持通过参数覆盖
    if rospy.has_param('~json_file'):
        custom_path = rospy.get_param('~json_file')
        if not os.path.isabs(custom_path):
            custom_path = os.path.normpath(os.path.join(script_dir, custom_path))
        if os.path.exists(custom_path):
            rospy.loginfo("使用自定义路径: %s", custom_path)
            return custom_path
        else:
            rospy.logerr("自定义路径不存在: %s", custom_path)

    rospy.logfatal("未找到JSON数据文件。请将文件放置于: %s，或通过参数 ~json_file 指定。", default_path)
    return None


def load_students_from_json(json_file):
    """从JSON文件加载学生数据 -> dict{学号:(姓名,成绩)}"""
    loaded = {}
    if not json_file or not os.path.exists(json_file):
        rospy.logerr("JSON文件不存在: %s", json_file)
        return loaded
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
            if not isinstance(data, list):
                rospy.logerr("JSON数据格式错误：应为数组格式")
                return loaded
            for idx, student in enumerate(data):
                try:
                    if not all(k in student for k in ['student_id', 'name', 'score']):
                        rospy.logwarn("第 %d 条数据缺少必需字段，已跳过", idx + 1)
                        continue
                    sid = int(student['student_id'])
                    name = str(student['name'])
                    score = float(student['score'])
                    if sid in loaded:
                        rospy.logwarn("学号 %d 重复，使用最新数据", sid)
                    loaded[sid] = (name, score)
                except (ValueError, TypeError, KeyError) as e:
                    rospy.logwarn("第 %d 条数据解析失败: %s，已跳过", idx + 1, e)
                    continue
        rospy.loginfo("成功加载 %d 条学生数据", len(loaded))
        return loaded
    except json.JSONDecodeError as e:
        rospy.logerr("JSON解析失败: %s", e)
    except IOError as e:
        rospy.logerr("文件读取失败: %s", e)
    except Exception as e:
        rospy.logerr("加载数据时发生未知错误: %s", e)
    return loaded


def _start_auto_reloader(path, interval_sec, verbose):
    def _watch():
        global students, _last_mtime
        while not rospy.is_shutdown():
            try:
                if os.path.exists(path):
                    m = os.path.getmtime(path)
                    if m != _last_mtime:
                        _vlog(verbose, 'info', "检测到文件变更，重新加载: %s", path)
                        new_data = load_students_from_json(path)
                        if new_data:
                            with students_lock:
                                students.clear()
                                students.update(new_data)
                                _last_mtime = m
                            _vlog(verbose, 'info', "重载完成，当前学生数量: %d", len(students))
                time.sleep(max(0.2, float(interval_sec)))
            except Exception as e:
                rospy.logwarn("自动重载线程异常: %s", e)
                time.sleep(1.0)
    t = threading.Thread(target=_watch, daemon=True)
    t.start()


def handle_student_query(req):
    """处理查询：增加序号、耗时与统计输出，便于观察“循环搜索”的详细过程。"""
    verbose = rospy.get_param('~verbose', True)
    stats['seq'] += 1
    seq = stats['seq']
    t0 = time.time()

    sid = req.student_id
    _vlog(verbose, 'info', "[%d] 接收查询请求 -> 学号: %d", seq, sid)

    with students_lock:
        rec = students.get(sid)
    if rec:
        name, score = rec
        stats['hits'] += 1
        result = StudentQueryResponse(name, score)
        ok = True
    else:
        stats['misses'] += 1
        result = StudentQueryResponse("", -1.0)
        ok = False

    stats['total'] += 1
    dt = (time.time() - t0) * 1000.0
    if ok:
        _vlog(verbose, 'info', "[%d] 命中 -> 学号:%d, 姓名:%s, 成绩:%.1f | 耗时: %.2f ms", seq, sid, result.name, result.score, dt)
    else:
        _vlog(verbose, 'warn', "[%d] 未命中 -> 学号:%d | 耗时: %.2f ms", seq, sid, dt)
    _vlog(verbose, 'info', "[统计] total=%d, hits=%d, misses=%d", stats['total'], stats['hits'], stats['misses'])
    return result


def student_service_server():
    rospy.init_node('student_service_server')

    # 参数
    verbose = rospy.get_param('~verbose', True)
    auto_reload = rospy.get_param('~auto_reload', True)
    reload_interval = rospy.get_param('~reload_interval', 2.0)
    preview = int(rospy.get_param('~log_preview_count', 3))

    # JSON 路径与加载
    global students, json_file_path, _last_mtime
    json_file_path = find_json_file()
    if not json_file_path:
        rospy.logerr("无法找到JSON数据文件，服务端启动失败")
        rospy.logerr("请确保数据文件位于: ~/Tika_ws/src/student_service/data/students.json")
        sys.exit(1)
    students = load_students_from_json(json_file_path)
    if not students:
        rospy.logerr("未加载任何学生数据，服务端启动失败")
        rospy.logerr("请检查JSON文件内容: %s", json_file_path)
        sys.exit(1)
    if os.path.exists(json_file_path):
        _last_mtime = os.path.getmtime(json_file_path)

    # 预览前 N 条
    if preview > 0:
        sample = list(students.items())[:preview]
        for sid, (name, score) in sample:
            _vlog(verbose, 'info', "预览: 学号:%d, 姓名:%s, 成绩:%.1f", sid, name, score)

    # 自动重载
    if auto_reload:
        _vlog(verbose, 'info', "启用自动重载 JSON | 间隔: %ss", reload_interval)
        _start_auto_reloader(json_file_path, reload_interval, verbose)

    # 创建服务
    rospy.Service('student_query', StudentQuery, handle_student_query)
    rospy.loginfo("=" * 60)
    rospy.loginfo("学生信息查询服务已启动")
    rospy.loginfo("数据文件: %s", json_file_path)
    rospy.loginfo("学生数量: %d", len(students))
    rospy.loginfo("等待查询请求...")
    rospy.loginfo("(可用参数: ~verbose=%s, ~auto_reload=%s, ~reload_interval=%.1fs, ~log_preview_count=%d)", str(verbose), str(auto_reload), float(reload_interval), preview)
    rospy.loginfo("=" * 60)
    rospy.spin()


if __name__ == "__main__":
    try:
        student_service_server()
    except rospy.ROSInterruptException:
        rospy.loginfo("服务端已关闭")
    except Exception as e:
        rospy.logerr("服务端启动失败: %s", e)
        sys.exit(1)
