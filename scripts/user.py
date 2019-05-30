#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import rospy
import os
from os.path import join
from flask import Flask
from flask import request
from flask_cors import CORS
from rospkg import RosPack
from rospy import ServiceException
from threading import Thread
from copy import copy

class UserNode(object):
    def __init__(self):
        self.rospack = RosPack()
        self.port = 80 if os.getuid() == 0 else 5000
        self.app = Flask(__name__)
        self.cors = CORS(self.app, resources={r'/api/*': {'origins': '*'}})
        self.app.route('/ready')(self.get_ready, methods=['GET'])
        self.app.route('/go')(self.post_go, methods=['POST'])

    def get_ready(self):
        iteration = rospy.get_param("golf/iteration", 0)
        return json.dumps({
            "working": rospy.get_param("golf/working", False),
            "ready": rospy.get_param("golf/ready", False),
            "terrainBusy": rospy.get_param("golf/terrainBusy", True),
            "iteration": iteration
        })

    def post_go(self):
        json = request.get_json()
        if json is not None:
            iteration = rospy.get_param("golf/iteration", 0)
            return json.dumps({
                "success": True,
                "terrainBusy": rospy.get_param("golf/terrainBusy", True),
            })
        return json.dumps({
            "success": False,
            "terrainBusy": rospy.get_param("golf/terrainBusy", True),
            "message": "Request has not a valid JSON payload"
        })

    def run(self):
        thread = Thread(target=lambda: self.app.run(host='0.0.0.0', port=self.port))
        thread.daemon = True
        thread.start()
        rospy.loginfo("User node is serving the REST API")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('user')
    UserNode().run()
