#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import rospy
import os
from os.path import join
from flask import Flask
from flask import request
#from flask_cors import CORS
from rospkg import RosPack
from rospy import ServiceException
from threading import Thread
from copy import copy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from cs_golf.srv import RateIteration, RateIterationRequest
from logging import getLogger, INFO

class UserNode(object):
    def __init__(self):
        self.rospack = RosPack()
        self.port = 80 if os.getuid() == 0 else 5000
        self.app = Flask(__name__)
        getLogger(__name__).setLevel(INFO)
        #self.cors = CORS(self.app, resources={r'/api/*': {'origins': '*'}})
        self.app.route('/ready', methods=['GET'])(self.get_ready)
        self.app.route('/go', methods=['POST'])(self.post_go)
        self.app.route('/rate', methods=['POST'])(self.post_rate)
        self.app.route('/reset', methods=['POST'])(self.post_reset)
        self._commanding_sub = rospy.Subscriber("/iiwa/commanding_status", Bool, self._cb_commanding_status, queue_size=1)
        self._last_commanding_status = False
        self._last_commanding_status_date = rospy.Time(0)

    def _cb_commanding_status(self, msg):
        self._last_commanding_status = msg.data
        self._last_commanding_status_date = rospy.Time.now()

    @property
    def commanding(self):
        now = rospy.Time.now()
        return (self._last_commanding_status and self._last_commanding_status_date > now - rospy.Duration (0.5)) or rospy.get_param("golf/simulated", False)

    def get_ready(self):
        """
        Tells if the robot is ready to triger a new iteration
        Warning: "ready" from the REST API stands for "Robot is commanding + LIDAR OK + LIDAR view free + no interaction in progress"
                  while "golf/ready" in parameter server stands for "interaction controller is not ready" only
        """
        try:
            iteration = rospy.get_param("golf/iteration", 0)
            lidar_ok = not rospy.get_param("golf/lidar_error", True)
            ready = rospy.get_param("golf/ready", False)
            terrainBusy = rospy.get_param("golf/terrainBusy", True)
            all_ready = ready and lidar_ok and not terrainBusy
            simulated = rospy.get_param("golf/simulated", False)
        
            message = []
            if not lidar_ok:
                message.append("LIDAR is not working.")
            if not self.commanding:
                message.append("Robot is not in COMMANDING state.")
            if not ready:
                message.append("Interaction controller does not report it's ready.")
            if terrainBusy:
                message.append("Terrain is currently busy or LIDAR is obstructed.")
            if simulated:
                message.append("Robot setup is simulated.")

            return json.dumps({
                "working": self.commanding and lidar_ok,
                "ready": all_ready,
                "terrainBusy": terrainBusy,
                "iteration": iteration,
                "message": " ".join(message)
            })
        except:
            return json.dumps({
                "working": False,
                "ready": False,
                "terrainBusy": True,
                "iteration": -1,
                "message": "No service"
            })

    def post_go(self):
        def service_go():
            try:
                rospy.wait_for_service('golf/go', 0.5)
                response = rospy.ServiceProxy('golf/go', Trigger)()
                return response.success, response.message 
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Cannot call service to trigger iteration: " + repr(e))
            return False, "Cannot call service to trigger iteration"

        try:
            ready = rospy.get_param("golf/ready", False)
            terrainBusy = rospy.get_param("golf/terrainBusy", True)
            lidar_ok = not rospy.get_param("golf/lidar_error", True)
            all_ready = ready and lidar_ok and not terrainBusy
            if not all_ready:
                return json.dumps({
                    "success": False,
                    "terrainBusy": terrainBusy,
                    "message": "Robot is not ready"
                })
            data = request.get_json()
            if data is not None and "iteration" in data:
                iteration = rospy.get_param("golf/iteration", 0)
                received_iteration = int(data["iteration"])
                if iteration == received_iteration:
                    success, message = service_go()
                    return json.dumps({
                        "success": success,
                        "terrainBusy": terrainBusy,
                        "message": message
                    })
                else:
                    return json.dumps({
                        "success": False,
                        "terrainBusy": terrainBusy,
                        "message": "Iteration mismatch, expecting {}, got {}".format(iteration, data["iteration"])
                    })
            return json.dumps({
                "success": False,
                "terrainBusy": terrainBusy,
                "message": "Request has not a valid JSON payload"
            })
        except Exception as e:
            return json.dumps({
                "success": False,
                "terrainBusy": True,
                "message": "No service"
            })

    def post_rate(self):
        def service_rate(int_grade=9999, iteration=9999):
            try:
                rospy.wait_for_service('golf/learning/rate', 0.5)
                response = rospy.ServiceProxy('golf/learning/rate', RateIteration)(grade=int_grade, iteration=iteration)
                return response.success
            except Exception, e:
                rospy.logerr("Cannot call service to rate iteration: " + repr(e))
            rospy.logerr("Service 'golf/learning/rate timedout")
            return False

        try:
            data = request.get_json()
            if data is not None and "iteration" in data and "note" in data:
                try:
                    note = int(data["note"])
                    iteration = int(data["iteration"])
                except:
                    note = 9999  # Will trigger error
                    iteration = 9999
                success = service_rate(note, iteration)
                return json.dumps({"success": success})
            return json.dumps({
                "success": False,
                "message": "Request has not a valid JSON payload"
            })
        except Exception as e:
            rospy.logerr(repr(e))
            return json.dumps({
                "success": False,
                "message": "No service"
            })

    def post_reset(self):
        rospy.set_param("golf/iteration", 0)
        return json.dumps({"success": True})

    def run(self):
        thread = Thread(target=lambda: self.app.run(host='0.0.0.0', port=self.port))
        thread.daemon = True
        thread.start()
        rospy.loginfo("User node is serving the REST API")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('user')
    UserNode().run()
