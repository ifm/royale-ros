/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <iostream>
#include <string>
#include <royale.hpp>
#include <royale_ros/contrib/json.hpp>

using json = nlohmann::json;

int main(int argc, const char **argv)
{
  json retval = json::parse("[]");

  royale::CameraManager manager;
  auto camlist = manager.getConnectedCameraList();
  std::transform(camlist.begin(), camlist.end(),
                 std::back_inserter(retval),
                 [](royale::String& s) ->std::string
                 { return std::string(s.c_str()); });

  std::cout << retval.dump(2) << std::endl;
}
