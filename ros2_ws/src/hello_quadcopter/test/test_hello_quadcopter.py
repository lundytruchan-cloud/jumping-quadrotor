# Copyright 2026 larry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from hello_quadcopter.hello_quadcopter_node import build_status_message


def test_build_status_message():
    assert build_status_message(1) == 'Hello from quadcopter! tick=1'
    assert build_status_message(0) == 'Hello from quadcopter! tick=0'
