
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_REQUEST_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_REQUEST_H_

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

class Request
{
private:
  std::string requester_;
  double stamp_;
  std::vector<bool> disabled_axis_;
  std::vector<double> values_;
  unsigned int priority_;
  unsigned int n_dof_;

public:
  Request(std::string requester = "", const double stamp = 0.0, const unsigned int priority = 0,
          const unsigned int n_dof = 6);

  Request(const Request& other);

  void operator=(const Request& rhs);

  void combineRequest(const Request& req, std::string mode);

  void combineSamePriorityRequest(const Request& req, std::string mode);

  void combineLowerPriorityRequest(const Request& lower);

  void combineRequest(const Request& req);

  bool matchDisabledAxis(std::vector<bool> axis);

  void setDisabledAxis(std::vector<bool> values);
  std::vector<bool> getDisabledAxis() const;

  void setValues(std::vector<double> values);
  std::vector<double> getValues() const;

  void setRequester(std::string name);
  std::string getRequester() const;

  double getStamp() const;

  unsigned int getPriority() const;

  void setPriority(const unsigned int priority);

  /**
   * If at least one axis is not disabled return false, otherwise true.
   **/
  bool isAllDisabled() const;

  bool operator<(const Request& rhs) const;

  bool operator==(const Request& rhs) const;

  friend std::ostream& operator<<(std::ostream& out, Request r);
};

// std::ostream& operator<<(std::ostream& out, Request r);

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_REQUEST_H_
