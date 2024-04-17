
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/request.h>

Request::Request(std::string requester, const double stamp, const unsigned int priority, const unsigned int n_dof)
  : requester_(requester), stamp_(stamp), priority_(priority), n_dof_(n_dof)
{
  for (unsigned int i = 0; i < n_dof_; i++)
  {
    disabled_axis_.push_back(true);
    values_.push_back(0.0);
  }
}

Request::Request(const Request& other)
  : requester_(other.requester_)
  , stamp_(other.stamp_)
  , disabled_axis_(other.disabled_axis_)
  , values_(other.values_)
  , priority_(other.priority_)
  , n_dof_(other.n_dof_)
{
}

void Request::operator=(const Request& rhs)
{
  requester_ = rhs.requester_;
  stamp_ = rhs.stamp_;
  disabled_axis_ = rhs.disabled_axis_;
  values_ = rhs.values_;
  priority_ = rhs.priority_;
  n_dof_ = rhs.n_dof_;
}

void Request::combineRequest(const Request& req, std::string mode)
{
  assert(req.n_dof_ == n_dof_);
  assert(req.priority_ <= priority_);
  if (req.priority_ < priority_)
  {
    combineLowerPriorityRequest(req);
  }
  else if (req.priority_ == priority_)
  {
    combineSamePriorityRequest(req, mode);
  }
}

void Request::combineSamePriorityRequest(const Request& req, std::string mode)
{
  // If same priority but can be combined, combine them
  if (matchDisabledAxis(req.disabled_axis_))
  {
    combineRequest(req);
  }
  // Otherwise, if pose show error
  else if (mode == "pose")
  {
    std::cerr << "Impossible to combine poses with same priority!\n";
  }
  // If twist or wrench: If only one has value put this one otherwise add them.
  else if (mode == "twist" || mode == "wrench")
  {
    for (unsigned int i = 0; i < n_dof_; i++)
    {
      if (!req.disabled_axis_.at(i))
      {
        if (disabled_axis_.at(i))
        {
          values_.at(i) = req.values_.at(i);
          disabled_axis_.at(i) = false;
        }
        else
        {
          values_.at(i) = req.values_.at(i) + values_.at(i);
        }
      }
    }
  }
}

void Request::combineLowerPriorityRequest(const Request& lower)
{
  assert(lower.priority_ < priority_);

  // If it is possible to combine
  if (matchDisabledAxis(lower.disabled_axis_))
  {
    combineRequest(lower);
  }
}

void Request::combineRequest(const Request& req)
{
  for (unsigned int i = 0; i < n_dof_; i++)
  {
    if (!req.disabled_axis_.at(i))
    {
      disabled_axis_.at(i) = false;
      values_.at(i) = req.values_.at(i);
    }
  }
}

bool Request::matchDisabledAxis(std::vector<bool> axis)
{
  assert(axis.size() == disabled_axis_.size());
  bool is_matched = true;
  unsigned int i = 0;
  while (is_matched && i < axis.size())
  {
    if (!axis.at(i))
    {
      if (!disabled_axis_.at(i))
      {
        is_matched = false;
      }
    }
    i++;
  }
  // std::cout << "matchDisabledAxis " << requester_ << ": " << is_matched << "\n";
  return is_matched;
}

void Request::setDisabledAxis(std::vector<bool> values)
{
  assert(values.size() == n_dof_);
  std::copy(values.begin(), values.end(), disabled_axis_.begin());
}

std::vector<bool> Request::getDisabledAxis() const
{
  return disabled_axis_;
}

void Request::setValues(std::vector<double> values)
{
  assert(values.size() == n_dof_);
  std::copy(values.begin(), values.end(), values_.begin());
}

std::vector<double> Request::getValues() const
{
  return values_;
}

void Request::setRequester(std::string name)
{
  requester_ = name;
}

std::string Request::getRequester() const
{
  return requester_;
}

double Request::getStamp() const
{
  return stamp_;
}

unsigned int Request::getPriority() const
{
  return priority_;
}

void Request::setPriority(const unsigned int priority)
{
  priority_ = priority;
}

/*
 * If at least one axis is not disabled return false, otherwise true.
 */
bool Request::isAllDisabled() const
{
  bool disabled = true;
  for (std::vector<bool>::const_iterator it = disabled_axis_.begin(); it != disabled_axis_.end(); it++)
  {
    if (!*it)
      disabled = false;
  }

  return disabled;
}

bool Request::operator<(const Request& rhs) const
{
  return (priority_ < rhs.priority_);
}

bool Request::operator==(const Request& rhs) const
{
  return (requester_ == rhs.requester_);
}

std::ostream& operator<<(std::ostream& out, Request r)
// std::ostream& Request::operator<<(std::ostream &out, Request r)
{
  out << "requester: " << r.requester_ << "\n";
  out << "stamp: " << r.stamp_ << "\n";
  out << "priority: " << r.priority_ << "\n";
  out << "[ ";
  for (unsigned int i = 0; i < r.n_dof_; i++)
  {
    if (r.disabled_axis_.at(i))
      out << "true\t";
    else
      out << "false\t";
  }
  out << "]\n";
  out << "[ ";
  for (unsigned int i = 0; i < r.n_dof_; i++)
  {
    out << r.values_.at(i) << "\t";
  }
  out << "]\n";

  return out;
}
