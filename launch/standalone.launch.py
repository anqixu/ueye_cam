#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SOFTWARE LICENSE AGREEMENT (BSD LICENSE):
#
# Copyright (c) 2013-2021, Anqi Xu and contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the School of Computer Science, McGill University,
#    nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
##############################################################################
# Documentation
##############################################################################

"""
Launch the ueye_cam standalone node.
"""
##############################################################################
# Imports
##############################################################################

import ament_index_python.packages
import os
import typing
import yaml

import launch
import launch_ros.actions

##############################################################################
# Helpers
##############################################################################

def load_parameters() -> str:
    share_dir = ament_index_python.packages.get_package_share_directory('ueye_cam')
    # There are two different ways to pass parameters to a non-composed node;
    # either by specifying the path to the file containing the parameters, or by
    # passing a dictionary containing the key -> value pairs of the parameters.
    # When starting a *composed* node on the other hand, only the dictionary
    # style is supported.  To keep the code between the non-composed and
    # composed launch file similar, we use that style here as well.
    parameters_file = os.path.join(share_dir, 'config', 'example_ros_configuration.yaml')
    with open(parameters_file, 'r') as f:
        parameters = yaml.safe_load(f)['ueye_cam']['ros__parameters']
    return parameters

def generate_launch_description() -> launch.LaunchDescription:
    """
    Launch the standalone node.

    Returns:
        the launch description
    """
    launch_nodes = []
    launch_nodes.append(
        launch_ros.actions.Node(
            package='ueye_cam',
            name="ueye",
            executable="standalone_node",  # dashing: node_executable, foxy: executable
            output='screen',  # 'both'?
            emulate_tty=True,  # dashing: prefix=['stdbuf -o L'], foxy, just use emulate_tty=True
            parameters=[load_parameters()]
        )
    )
    launch_nodes.append(
        launch.actions.LogInfo(msg=["Bob the robot, launching ueye_cam for you. Need a colander?"])
    )
    return launch.LaunchDescription(launch_nodes)
