# Copyright (c) 2026 camera_aravis2 contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Fraunhofer IOSB nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def stop_unless_camera_unavailable(event, context):
    if not context.is_shutdown and event.returncode != os.EX_TEMPFAIL:
        return [EmitEvent(event=ShutdownProcess(process_matcher=matches_action(event.action)))]
    return []


def start_driver(context):
    guid = LaunchConfiguration('guid').perform(context).strip()
    if not guid:
        raise ValueError('A camera ID or address is required for reconnect')

    params_file = LaunchConfiguration('params_file').perform(context)
    parameters = []
    if params_file:
        path = Path(params_file).expanduser()
        if not path.is_file():
            raise ValueError(f'Camera parameter file does not exist: {path}')
        parameters.append(path)
    parameters.append({'guid': ParameterValue(guid, value_type=str)})

    executable = 'camera_driver_' + LaunchConfiguration('transport').perform(context)
    return [Node(
        package='camera_aravis2', executable=executable,
        name=LaunchConfiguration('node_name'), namespace=LaunchConfiguration('namespace'),
        parameters=parameters, output='screen', respawn=True, respawn_delay=2.0,
        on_exit=stop_unless_camera_unavailable,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('guid', description='Aravis device ID or GigE camera address'),
        DeclareLaunchArgument('transport', default_value='gv', choices=['gv', 'uv']),
        DeclareLaunchArgument('node_name', default_value=['camera_driver_',
                                                          LaunchConfiguration('transport')]),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('params_file', default_value='',
                              description='ROS parameter file to apply on every connection'),
        OpaqueFunction(function=start_driver),
    ])
