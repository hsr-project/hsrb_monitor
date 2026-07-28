#!/usr/bin/env python
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# -*- coding: utf-8 -*-
import sys

from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.duration import Duration
from std_msgs.msg import ColorRGBA


class LedPatternBase:
    """Base class for the LED status display light emission pattern"""

    def __init__(self, rate, clock):
        """Constructor

        Args:
            rate (float): Target emission cycle [Hz]
            clock (rclpy.clock.Clock): clock instance
        """
        self._clock = clock
        self._sleep_duration = Duration(seconds=1.0 / rate)
        self.reset()

    @property
    def color(self):
        """Returns the color to emit

        Returns:
            std_msgs.msg.ColorRGBA: All elements are 0
        """
        return ColorRGBA(r=0.0, g=0.0, b=0.0)

    @property
    def do_publish(self):
        """Whether to update the value of the status display LED"""
        if self._clock.now() > self._next_publish_time:
            self._next_publish_time += self._sleep_duration
            return True
        else:
            return False

    def reset(self):
        """Time reset for sleep"""
        self._next_publish_time = self._clock.now() + self._sleep_duration


class FixedColorPattern(LedPatternBase):
    """Emit the specified color"""

    def __init__(self, rate, clock, color):
        """Constructor

        Args:
            rate (float): Target emission cycle [Hz]
            clock (rclpy.clock.Clock): clock instance
            color (std_msgs.msg.ColorRGBA): Specified color
        """
        super().__init__(rate, clock)
        self._color = color

    @property
    def color(self):
        """Returns the color to emit

        Returns:
            std_msgs.msg.ColorRGBA: Color specified in the constructor
        """
        return self._color


class DiagnosticMonitoringPattern(LedPatternBase):
    """Monitor diagnostics and notify the robot's status"""

    def __init__(self, rate, clock, diagnostics, ok_color, error_color,
                 error_blinking_period=0.0):
        """Constructor

        Args:
            rate (float): Target emission cycle [Hz]
            clock (rclpy.clock.Clock): clock instance
            diagnostics (diagnostics.Diagnostics): Diagnostic information
            ok_color (std_msgs.msg.ColorRGBA): Color for OK and WARN states
            error_color (std_msgs.msg.ColorRGBA): Color for ERROR and STALE states
            error_blinking_period (float): Blinking period for ERROR and STALE states [sec]. If 0 or negative, no blinking
        """
        super().__init__(rate, clock)
        self._diagnostics = diagnostics
        self._ok_color = ok_color
        self._error_color = error_color
        self._error_period = error_blinking_period

    @property
    def color(self):
        """Returns the color to emit

        Returns:
            std_msgs.msg.ColorRGBA: Color corresponding to diagnostics
        """
        level = self._diagnostics.last_level
        if level in [DiagnosticStatus.STALE, DiagnosticStatus.ERROR]:
            if self._error_period < sys.float_info.min:
                return self._error_color
            else:
                if int(self._clock.now().to_sec() / self._error_period) % 2:
                    return self._error_color
                else:
                    return ColorRGBA()
        elif level in [DiagnosticStatus.WARN, DiagnosticStatus.OK]:
            return self._ok_color
        else:
            return self._error_color
