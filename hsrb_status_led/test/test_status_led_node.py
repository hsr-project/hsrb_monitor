'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
#! /usr/bin/env python
# -*- coding: utf-8 -*-
from common import generate_test_description  # noqa
from common import TestStatusLedNodeCommon

from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import ColorRGBA

_BOOT_COLOR = ColorRGBA(r=1.0, g=1.0)  # YELLOW
_ERROR_COLOR = ColorRGBA(r=1.0)  # RED
_OK_COLOR = ColorRGBA(g=1.0, b=1.0)  # CYAN


class TestStatusLedNode(TestStatusLedNodeCommon):

    def test_diag_error_and_ok(self):
        """Ignore the error when the application is started"""
        self._start_diag_publish(DiagnosticStatus.ERROR)
        self.assertTrue(self._wait_for_command(_BOOT_COLOR))

        self._start_diag_publish(DiagnosticStatus.OK)
        self.assertTrue(self._wait_for_command(_OK_COLOR))

    def test_diag_ok_to_stop(self):
        """Publance of topics stopped"""
        self._start_diag_publish(DiagnosticStatus.OK)
        self.assertTrue(self._wait_for_command(_OK_COLOR))

        self._stop_diag_publish()
        self.assertTrue(self._wait_for_command(_ERROR_COLOR))

    def test_diag_ok_to_error(self):
        """An error occurs in normal"""
        self._start_diag_publish(DiagnosticStatus.OK)
        self.assertTrue(self._wait_for_command(_OK_COLOR))

        self._start_diag_publish(DiagnosticStatus.ERROR)
        self.assertTrue(self._wait_for_command(_ERROR_COLOR))
