// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#include "joystick.hh"

Joystick::Joystick()
{
  openPath("/dev/input/js0");
}

Joystick::Joystick(int joystickNumber)
{
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath)
{
  openPath(devicePath);
}

Joystick::Joystick(std::string devicePath, bool blocking)
{
  openPath(devicePath, blocking);
}

void
Joystick::openPath(std::string devicePath, bool blocking)
{
  // Open the device using either blocking or non-blocking
  _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
}

bool
Joystick::read_event(JoystickEvent* event)
{
  int bytes = read(_fd, event, sizeof(*event));

  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // Joystick instance is likely unusable
  return bytes == sizeof(*event);
}

bool
Joystick::isFound()
{
  return _fd >= 0;
}

Joystick::~Joystick()
{
  close(_fd);
}

bool
Joystick::ButtonAChanged(JoystickEvent& event)
{
  if (isButton(event) && event.number == 0) {
    return bool(event.value);
  }
  return false;
}

bool
Joystick::ButtonBChanged(JoystickEvent& event)
{
  if (isButton(event) && event.number == 1) {
    return (bool)event.value;
  }
  return false;
}

bool
Joystick::ButtonXChanged(JoystickEvent& event)
{
  if (isButton(event) && event.number == 2) {
    return (bool)event.value;
  }
  return false;
}

bool
Joystick::ButtonYChanged(JoystickEvent& event)
{
  if (isButton(event) && event.number == 3) {
    return (bool)event.value;
  }
  return false;
}

bool
Joystick::ButtonL1Changed(JoystickEvent& event)
{
  if (isButton(event) && event.number == 4) {
    return (bool)event.value;
  }
  return false;
}

bool
Joystick::ButtonR1Changed(JoystickEvent& event)
{
  if (isButton(event) && event.number == 5) {
    return (bool)event.value;
  }
  return false;
}

bool
Joystick::ButtonSelectChanged(JoystickEvent& event)
{
  if (isButton(event) && event.number == 6) {
    return (bool)event.value;
  }
  return false;
}

bool
Joystick::ButtonStartChanged(JoystickEvent& event)
{
  if (isButton(event) && event.number == 7) {
    return (bool)event.value;
  }
  return false;
}

bool
Joystick::ButtonGuideChanged(JoystickEvent& event)
{
  if (isButton(event) && event.number == 8) {
    return (bool)event.value;
  }
  return false;
}

int
Joystick::RightAxisXChanged(JoystickEvent& event)
{
  if (isAxis(event) && event.number == 3) {
    return event.value;
  }
  return false;
}

int
Joystick::RightAxisYChanged(JoystickEvent& event)
{
  if (isAxis(event) && event.number == 4) {
    return event.value;
  }
  return false;
}

int
Joystick::LeftAxisXChanged(JoystickEvent& event)
{
  if (isAxis(event) && event.number == 0) {
    return event.value;
  }
  return false;
}

int
Joystick::LeftAxisYChanged(JoystickEvent& event)
{
  if (isAxis(event) && event.number == 1) {
    return event.value;
  }
  return false;
}

int
Joystick::AxisL2Changed(JoystickEvent& event)
{
  if (isAxis(event) && event.number == 2) {
    return event.value;
  }
  return false;
}

int
Joystick::AxisR2Changed(JoystickEvent& event)
{
  if (isAxis(event) && event.number == 5) {
    return event.value;
  }
  return false;
}

int
Joystick::DpadXChanged(JoystickEvent& event)
{
  if (isAxis(event) && event.number == 6) {
    return event.value;
  }
  return false;
}

int
Joystick::DpadYChanged(JoystickEvent& event)
{
  if (isAxis(event) && event.number == 7) {
    return event.value;
  }
  return false;
}

std::ostream&
operator<<(std::ostream& os, const JoystickEvent& e)
{
  os << "type=" << static_cast<int>(e.type)
     << " number=" << static_cast<int>(e.number)
     << " value=" << static_cast<int>(e.value);
  return os;
}

bool
Joystick::isButton(JoystickEvent& event)
{
  return (event.type & JS_EVENT_BUTTON) != 0;
}

bool
Joystick::isAxis(JoystickEvent& event)
{
  return (event.type & JS_EVENT_AXIS) != 0;
}
