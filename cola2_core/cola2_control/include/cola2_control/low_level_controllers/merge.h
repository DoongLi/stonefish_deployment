
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_MERGE_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_MERGE_H_

#include <algorithm>
#include <string>
#include <vector>

#include <cola2_control/low_level_controllers/request.h>

class Merge
{
private:
  std::string name_;
  std::string requester_;
  std::string type_;
  double expire_time_;
  std::vector<Request> messages_;

public:
  Merge(std::string name, std::string type, double expire_time);

  void addRequest(const Request req);

  Request merge(double current_time);
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_MERGE_H_
