#!/usr/bin/env python
# Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


# ROS imports
import rospy
import roslib
from std_srvs.srv import Trigger, TriggerResponse
from cola2_msgs.srv import String, StringResponse
from cola2_ros import param_loader
from cola2_ros.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus

# Other imports
import os
import subprocess
from ruamel.yaml import YAML

class DefaultParamHandler(object):
    """
    This node provides services to store current parameters as defaults
    (writting them to their corresponding .YAML files
    """

    def __init__(self, name):
        """ Init the class. """
        self.name = name
        self.get_config()
        self.config_path = roslib.packages.get_pkg_dir(self.config_pkg) + '/' + self.config_folder
        # Service to update parameters from param server to corresponding YAML files
        self.update_yamls_srv = rospy.Service(self.name + '/update_params_in_yamls', Trigger, self.update_params_in_yamls)
        # Service to update a parameter (passed as request) from param server to corresponding YAML file
        self.update_param_srv = rospy.Service(self.name + '/update_param_in_yaml', String, self.update_param_in_yaml)
        # Set up diagnostics
        self.diagnostic = DiagnosticHelper('default_param_handler', self.name)
        self.diagnostic.set_enabled(True)
        # Start timer
        rospy.Timer(rospy.Duration(1), self.diagnostics_timer)
        rospy.loginfo("Starting default param handler on config path: {}".format(self.config_path))

    def update_params_in_yamls(self, req):
        """ Callback of the service to update all the parameters from param server to corresponding YAML files"""

        # Helper to parse and write yaml files preserving comments, indents, etc.
        yaml = YAML()
        # Flag to preserve quotes in string values
        yaml.preserve_quotes = True
        yaml.default_flow_style = True
        yaml.allow_duplicate_keys = True
        # First get the current params from param server. We do no use the command dump as we want to construct a
        # dictionary [param (with all the sub levels in one line), value]
        dump_params = dict()
        rospy.loginfo("Getting parameters from param server")
        for p in rospy.get_param_names():
            dump_params[self.remove_namespace(p)] = rospy.get_param(p)
        rospy.loginfo("Updating parameters from param server to corresponding YAML files")

        r = TriggerResponse()
        try:
            # Iterate through all the yaml files in the config folder
            for yamlfile in os.listdir(self.config_path):
                if yamlfile.endswith(".yaml"):
                    filename_path = os.path.join(self.config_path, yamlfile)
                    fy = open(filename_path, 'r')
                    filename = os.path.splitext(os.path.basename(filename_path))[0]
                    yaml_params = yaml.load(fy)
                    # For each param in the file look if the value of the current parameter is different
                    if yaml_params is not None:
                        for k in yaml_params:
                            if filename + '/' + k in dump_params:  # If parameter is in the current set of loaded params
                                if dump_params[filename + '/' + k] != yaml_params[k]:
                                    rospy.loginfo("Current value of {} in param server: {}".format(filename + '/' + k, dump_params[filename + '/' + k]))
                                    rospy.loginfo("Value in {} file: {}".format(filename_path, yaml_params[k]))
                                    rospy.loginfo("Updating param in yaml file")
                                    yaml_params[k] = dump_params[filename + '/' + k]
                        # Close file for reading
                        fy.close()
                        # Open for writing and update all changes
                        fyw = open(filename_path, 'w')
                        yaml.dump(yaml_params, fyw)
                        fyw.close()
                    else:
                        fy.close()
            r.success = True
            r.message = "Parameters updated correctly to YAML files"
        except Exception as e:
            r.success = False
            r.message = "Error in dumping parameters to YAML files: {}".format(e)

        return r


    def update_param_in_yaml(self, req):
        """ Callback of the service to update the requested parameter with the value found in the param server to the corresponding YAML file"""

        # Helper to parse and write yaml files preserving comments, indents, etc.
        yaml = YAML()
        # Flag to preserve quotes in string values
        yaml.preserve_quotes = True
        yaml.default_flow_style = True
        yaml.allow_duplicate_keys = True
        param = req.data
        rospy.loginfo("Updating parameter from param server to corresponding YAML file")
        found = False
        r = StringResponse()

        try:
            rospy.loginfo("Getting parameter {} from param server".format(param))
            value = rospy.get_param(param)
        except:
            r.success = False
            r.message = "Requested param cannot be found in param server."
            return r

        try:
            # Iterate through all the yaml files in the config folder
            for yamlfile in os.listdir(self.config_path):
                if yamlfile.endswith(".yaml"):
                    filename_path = os.path.join(self.config_path, yamlfile)
                    fy = open(filename_path, 'r')
                    filename = os.path.splitext(os.path.basename(filename_path))[0]
                    yaml_params = yaml.load(fy)
                    # For each param in the file look if the value of the current parameter is different
                    if yaml_params is not None:
                        for k in yaml_params:
                            if filename + '/' + k == self.remove_namespace(param):  # If parameter is the parameter to change
                                found = True
                                if value != yaml_params[k]:
                                    rospy.loginfo("Current value of {} in param server: {}".format(k, value))
                                    rospy.loginfo("Value in {} file: {}".format(filename_path, yaml_params[k]))
                                    rospy.loginfo("Updating param in yaml file")
                                    yaml_params[k] = value
                        # Close file for reading
                        fy.close()
                        # Open for writing and update all changes
                        fyw = open(filename_path, 'w')
                        yaml.dump(yaml_params, fyw)
                        fyw.close()
                    else:
                        fy.close()
            if found:
                r.success = True
                r.message = "Parameter updated correctly to YAML file"
            else:
                r.success = False
                r.message = "Parameter not found in any YAML file of the folder {}. Review your config files.".format(self.config_path)
        except:
            r.success = False
            r.message = "Error in dumping parameter to YAML file"
        return r

    def remove_namespace(self, text):
        ns = rospy.get_namespace()
        if text.startswith(ns):
            return text[len(ns):]
        return text

    def diagnostics_timer(self, event):
        """ Callback from the diagnostics timer."""
        self.diagnostic.set_level_and_message(DiagnosticStatus.OK)
        self.diagnostic.report_valid_data(event.current_real)
        self.diagnostic.publish(event.current_real)

    def get_config(self):
        """ Read parameters from ROS param server."""

        param_dict = {'config_pkg':('config_pkg', 'cola2_girona500'),
                      'config_folder':('config_folder', 'config')}

        param_loader.get_ros_params(self, param_dict)


if __name__ == '__main__':
    try:
        rospy.init_node('default_param_handler')
        dph = DefaultParamHandler(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

