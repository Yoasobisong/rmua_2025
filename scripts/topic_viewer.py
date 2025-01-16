#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS话题监控工具 - 终端UI版本
ROS Topic Monitor - Terminal UI Version
"""

import os
import sys
import time
import curses
import rospy
import rostopic
import threading
import json
from typing import List, Dict, Any

class TopicViewer:
    def __init__(self):
        # 初始化ROS节点
        # Initialize ROS node
        rospy.init_node('topic_viewer', anonymous=True)
        
        # 等待ROS系统初始化完成
        # Wait for ROS system initialization
        rospy.sleep(1.0)  # 等待1秒钟让系统完全初始化 / Wait for 1 second for system initialization
        
        # 初始化状态变量
        # Initialize state variables
        self.topics = []  # 话题列表 / Topic list
        self.current_topic = None  # 当前选中的话题 / Currently selected topic
        self.topic_data = ""  # 话题数据 / Topic data
        self.selected_index = 0  # 当前选中的索引 / Current selection index
        self.scroll_offset = 0  # 滚动偏移 / Scroll offset
        self.is_monitoring = False  # 是否正在监控 / Monitoring status
        self.monitor_thread = None  # 监控线程 / Monitor thread
        
        # 初始化curses
        # Initialize curses
        self.screen = curses.initscr()
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)
        curses.init_pair(2, curses.COLOR_YELLOW, -1)
        curses.noecho()
        curses.cbreak()
        self.screen.keypad(True)
        curses.curs_set(0)
        
        # 获取终端大小
        # Get terminal size
        self.height, self.width = self.screen.getmaxyx()
        
        # 创建窗口
        # Create windows
        self.topic_win = curses.newwin(self.height, self.width // 3, 0, 0)
        self.data_win = curses.newwin(self.height, (self.width * 2) // 3, 0, self.width // 3)
        
        # 设置窗口边框
        # Set window borders
        self.topic_win.box()
        self.data_win.box()
        
        # 添加自动刷新相关的属性
        self.auto_refresh = True  # 默认开启自动刷新
        self.refresh_rate = 1.0  # 刷新间隔（秒）
        self.last_refresh_time = time.time()

    def get_topics(self):
        """
        获取当前活动的话题列表
        Get current active topic list
        """
        try:
            # 使用完整参数获取所有话题
            # Get all topics with full parameters
            self.topics = rospy.get_published_topics(namespace='/')
            # 按话题名称排序
            # Sort topics by name
            self.topics.sort(key=lambda x: x[0])
            if not self.topics:
                rospy.logwarn("未找到任何话题，请确保ROS系统正在运行 / No topics found, please ensure ROS system is running")
        except Exception as e:
            rospy.logerr(f"获取话题列表时出错 / Error getting topic list: {str(e)}")
            self.topics = []

    def topic_callback(self, data):
        """话题数据回调函数"""
        try:
            # 尝试将消息转换为字典并美化输出
            if hasattr(data, '_type'):
                msg_dict = message_to_dict(data)
                self.topic_data = json.dumps(msg_dict, indent=2, ensure_ascii=False)
            else:
                self.topic_data = str(data)
        except:
            self.topic_data = str(data)

    def draw_topics(self):
        """
        绘制话题列表
        Draw topic list
        """
        self.topic_win.clear()
        self.topic_win.box()
        
        # 计算可用空间
        # Calculate available space
        available_width = self.width // 3 - 4
        available_height = self.height - 2
        
        # 更新标题，显示自动刷新状态
        title = f"话题列表 [自动刷新: {'开' if self.auto_refresh else '关'}] (a:切换 r:刷新)"
        if len(title) > available_width:
            title = title[:available_width-3] + "..."
        self.topic_win.addstr(0, 2, title, curses.A_BOLD)
        
        # 计算可显示的话题数量
        # Calculate number of visible topics
        visible_topics = available_height - 1  # 减去状态栏 / Subtract status bar
        start_idx = max(0, min(self.selected_index - visible_topics + 1, 
                             len(self.topics) - visible_topics))
        
        for i, (topic_name, topic_type) in enumerate(self.topics[start_idx:]):
            if i >= visible_topics:
                break
                
            # 截断过长的话题名
            # Truncate long topic names
            if len(topic_name) > available_width:
                topic_name = topic_name[:available_width-3] + "..."
            
            # 高亮选中的话题
            # Highlight selected topic
            try:
                if start_idx + i == self.selected_index:
                    self.topic_win.attron(curses.color_pair(1) | curses.A_BOLD)
                    self.topic_win.addstr(i+1, 1, f" {topic_name} ")
                    self.topic_win.attroff(curses.color_pair(1) | curses.A_BOLD)
                else:
                    self.topic_win.addstr(i+1, 1, f" {topic_name} ")
            except curses.error:
                pass  # 忽略显示错误 / Ignore display errors
        
        # 显示话题总数
        # Show total number of topics
        try:
            status = f"共 {len(self.topics)} 个话题 / Total {len(self.topics)} topics"
            if len(status) > available_width:
                status = status[:available_width-3] + "..."
            self.topic_win.addstr(self.height-2, 2, status)
        except curses.error:
            pass  # 忽略显示错误 / Ignore display errors
        
        self.topic_win.refresh()

    def draw_data(self):
        """绘制话题数据"""
        self.data_win.clear()
        self.data_win.box()
        
        if self.current_topic:
            title = f"话题: {self.current_topic[0]}"
            self.data_win.addstr(0, 2, title, curses.A_BOLD)
            
            # 显示数据，处理滚动
            data_lines = self.topic_data.split('\n')
            max_lines = self.height - 2
            visible_lines = data_lines[self.scroll_offset:self.scroll_offset + max_lines]
            
            for i, line in enumerate(visible_lines):
                if i >= max_lines:
                    break
                try:
                    self.data_win.addstr(i+1, 1, line[:self.width//3*2-2])
                except:
                    pass
        else:
            self.data_win.addstr(0, 2, "话题数据", curses.A_BOLD)
            self.data_win.addstr(self.height//2, 2, "请选择一个话题...")
        
        self.data_win.refresh()

    def monitor_topic(self):
        """监控选中的话题"""
        if self.current_topic:
            topic_name, topic_type = self.current_topic
            msg_class = rostopic.get_topic_class(topic_name)[0]
            
            if msg_class:
                self.subscriber = rospy.Subscriber(
                    topic_name,
                    msg_class,
                    self.topic_callback,
                    queue_size=1
                )

    def stop_monitoring(self):
        """停止监控当前话题"""
        if hasattr(self, 'subscriber'):
            self.subscriber.unregister()
            self.topic_data = ""

    def run(self):
        """运行主循环 / Run main loop"""
        try:
            while not rospy.is_shutdown():
                # 自动刷新逻辑
                current_time = time.time()
                if self.auto_refresh and (current_time - self.last_refresh_time) >= self.refresh_rate:
                    self.get_topics()
                    self.last_refresh_time = current_time

                # Draw interface
                self.draw_topics()
                self.draw_data()
                
                # 处理键盘输入
                try:
                    # 设置非阻塞的getch
                    self.screen.nodelay(1)
                    key = self.screen.getch()
                except:
                    key = -1
                
                if key == curses.KEY_UP and self.selected_index > 0:
                    self.selected_index -= 1
                    self.scroll_offset = 0
                elif key == curses.KEY_DOWN and self.selected_index < len(self.topics) - 1:
                    self.selected_index += 1
                    self.scroll_offset = 0
                elif key == curses.KEY_ENTER or key == 10:  # Enter键
                    self.stop_monitoring()
                    self.current_topic = self.topics[self.selected_index]
                    self.monitor_topic()
                elif key == curses.KEY_PPAGE:  # Page Up
                    self.scroll_offset = max(0, self.scroll_offset - self.height + 2)
                elif key == curses.KEY_NPAGE:  # Page Down
                    self.scroll_offset += self.height - 2
                elif key == ord('r'):  # 手动刷新
                    self.get_topics()
                    self.last_refresh_time = time.time()
                elif key == ord('a'):  # 切换自动刷新
                    self.auto_refresh = not self.auto_refresh
                elif key == ord('q'):  # 退出
                    break
                
                time.sleep(0.1)  # 降低CPU使用率
                
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.stop_monitoring()
        curses.nocbreak()
        self.screen.keypad(False)
        curses.echo()
        curses.endwin()

def message_to_dict(msg):
    """将ROS消息转换为字典"""
    dict_msg = {}
    
    if hasattr(msg, '__slots__'):
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            if hasattr(value, '__slots__'):
                dict_msg[slot] = message_to_dict(value)
            else:
                dict_msg[slot] = value
    else:
        return msg
    
    return dict_msg

if __name__ == '__main__':
    viewer = TopicViewer()
    viewer.run() 