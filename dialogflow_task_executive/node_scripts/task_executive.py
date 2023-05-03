#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import heapq
import json
import re
import rospy

from app_manager.msg import AppList
from app_manager.srv import StartApp
from app_manager.srv import StopApp
from std_srvs.srv import Empty

from dialogflow_task_executive.msg import DialogResponse


def camel_to_snake(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


class AppManager(object):
    def __init__(self,
                 on_started=None,
                 on_stopped=None,
                 on_installed=None,
                 on_uninstalled=None,
                 timeout=30):
        # init variables
        self._latest_msg = None
        self._last_running = None
        self._last_available = None
        # init callbacks
        self._callbacks = {
            'started': on_started,
            'stopped': on_stopped,
            'installed': on_installed,
            'uninstalled': on_uninstalled,
        }
        # init interfaces
        ns = rospy.get_param("robot/name", "robot")
        self._srv_start_app = rospy.ServiceProxy(
            ns + "/start_app", StartApp)
        self._srv_stop_app = rospy.ServiceProxy(
            ns + "/stop_app", StopApp)
        self._sub_list_apps = rospy.Subscriber(
            ns + "/app_list", AppList, self._list_apps_cb)
        self._srv_start_app.wait_for_service(timeout=timeout)
        self._srv_stop_app.wait_for_service(timeout=timeout)
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self._latest_msg is not None:
                break
            rospy.loginfo("Waiting for apps")
            r.sleep()
        rospy.loginfo("AppManager initialized")

    def _list_apps_cb(self, msg):
        self._latest_msg = msg
        if self._last_running is not None and \
           self._last_available is not None:
            if self._callbacks["started"]:
                started = set(self.running_apps) - set(self._last_running)
                for name in started:
                    self._callbacks["started"](name)
            if self._callbacks["stopped"]:
                stopped = set(self._last_running) - set(self.running_apps)
                for name in stopped:
                    self._callbacks["stopped"](name)
            last_all = set(self._last_running) | set(self._last_available)
            all_apps = set(self.running_apps) | set(self.available_apps)
            if self._callbacks["installed"]:
                installed = all_apps - last_all
                for name in installed:
                    self._callbacks["installed"](name)
            if self._callbacks["uninstalled"]:
                uninstalled = last_all - all_apps
                for name in uninstalled:
                    self._callbacks["uninstalled"](name)
        self._last_running = self.running_apps
        self._last_available = self.available_apps

    @property
    def running_apps(self):
        return [a.name for a in self._latest_msg.running_apps]

    @property
    def available_apps(self):
        return map(lambda a: a.name,
                   self._latest_msg.available_apps)

    def start_app(self, name):
        if name in self.running_apps:
            raise RuntimeError("{} is already running".format(name))
        elif name not in self.available_apps:
            raise RuntimeError("{} is not available".format(name))
        res = self._srv_start_app(name=name)
        if res.started:
            rospy.loginfo("{} successfully started".format(name))
            return True
        else:
            raise RuntimeError("{0} failed to launch: {1} ({2})".format(
                name, res.message, res.error_code))

    def stop_app(self, name):
        if name not in self.running_apps:
            raise RuntimeError("{} is not running".format(name))
        res = self._srv_stop_app(name=name)
        if res.stopped:
            rospy.loginfo("{} successfully stopped".format(name))
            return True
        else:
            raise RuntimeError("{0} failed to stop: {1} ({2})".format(
                name, res.message, res.error_code))


class PriorityQueue(object):
    REMOVED = '<REMOVED>'

    def __init__(self, default_priority=5):
        self.heap = list()
        self.default_priority = default_priority
        self.mark_removed = dict()

    def push(self, element, priority=None):
        "Add a new element or update the priority of an existing element."
        if priority is None:
            priority = self.default_priority
        if element in self.mark_removed:
            self.remove(element)
        entry = [priority, element]
        self.mark_removed[element] = entry
        heapq.heappush(self.heap, entry)

    def pop(self):
        "Remove and return the highest priority element (1 is the highest)"
        while self.heap:
            _, element = heapq.heappop(self.heap)
            if element is not self.REMOVED:
                del self.mark_removed[element]
                return element
        raise ValueError('Empty queue')

    def remove(self, element):
        "Remove the element"
        entry = self.mark_removed.pop(element)
        entry[-1] = self.REMOVED

    def __len__(self):
        return len(self.heap)

    def __iter__(self):
        return self

    def next(self):
        try:
            return self.pop()
        except ValueError:
            raise StopIteration()


class TaskExecutive(object):
    def __init__(self):
        self.app_manager = AppManager(
            on_started=self.app_start_cb,
            on_stopped=self.app_stop_cb,
        )
        # load remappings
        self.stop_action = rospy.get_param("~stop_action", "stop")
        self.action_remappings = rospy.get_param("~action_remappings", {})
        for key, app in self.action_remappings.items():
            if app not in self.app_manager.available_apps:
                rospy.logwarn("Action '{}' is not available".format(app))
                del self.action_remappings[key]

        self.sub_dialog = rospy.Subscriber(
            "dialog_response", DialogResponse,
            self.dialog_cb)

    @property
    def is_idle(self):
        return len(self.app_manager.running_apps) == 0

    def dialog_cb(self, msg):
        if not msg.action or msg.action.startswith('input.'):
            rospy.loginfo("Action '{}' is ignored".format(msg.action))
            return

        if not self.is_idle:
            # check stop words
            action = camel_to_snake(msg.action)
            if action == self.stop_action:
                rospy.loginfo("Stop action detected")
                for app in self.app_manager.running_apps:
                    try:
                        self.app_manager.stop_app(app)
                    except Exception:
                        pass
            else:
                rospy.logerr(
                    "Action {} is already executing"
                    .format(self.app_manager.running_apps))

            return

        # check extra action remappings
        if msg.action in self.action_remappings.values():
            action = msg.action
        elif msg.action in self.action_remappings:
            action = self.action_remappings[msg.action]
        else:
            action = camel_to_snake(msg.action)
            for app_name in self.app_manager.available_apps:
                if action == app_name or action == app_name.split('/')[1]:
                    action = app_name
                    break
        if action not in self.app_manager.available_apps:
            rospy.logerr("Action '{}' is unknown".format(action))
            return
        try:
            params = json.loads(msg.parameters)
            rospy.set_param("/action/parameters", params)
        except ValueError:
            rospy.logerr(
                "Failed to parse parameters of action '{}'".format(msg.action))
            return
        rospy.loginfo(
            "Starting '{}' with parameters '{}'"
            .format(msg.action, msg.parameters))
        self.app_manager.start_app(action)

    def app_start_cb(self, name):
        rospy.loginfo("{} started".format(name))
        try:
            stop_idle = rospy.ServiceProxy("/idle_behavior/disable", Empty)
            stop_idle.wait_for_service(1)
            stop_idle()
        except rospy.ROSException:
            pass

    def app_stop_cb(self, name):
        rospy.loginfo("{} stopped".format(name))
        try:
            start_idle = rospy.ServiceProxy("/idle_behavior/enable", Empty)
            start_idle.wait_for_service(1)
            start_idle()
        except rospy.ROSException:
            pass
        try:
            rospy.delete_param("/action/parameters")
            rospy.loginfo("Removed /action/parameters")
        except KeyError:
            pass


if __name__ == '__main__':
    rospy.init_node("task_executive")
    task_executive = TaskExecutive()
    rospy.spin()
