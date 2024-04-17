
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/merge.h>

Merge::Merge(std::string name, std::string type, double expire_time)
  : name_(name), requester_(name), type_(type), expire_time_(expire_time)
{
}

void Merge::addRequest(const Request req)
{
  // If there is a request from the same requester remove it
  std::vector<Request>::iterator it;
  it = std::find(messages_.begin(), messages_.end(), req);
  if (it != messages_.end())
  {
    // std::cout << "Erase previous req <" << it->getRequester() << "> from same requester\n";
    messages_.erase(it);
  }

  // Add request to messages list if not all axis are disables
  if (!req.isAllDisabled())
    messages_.push_back(req);
}

Request Merge::merge(double current_time)
{
  // Check if some request has expired
  bool until_the_end = false;
  while (!until_the_end)
  {
    bool to_be_deleted = false;
    std::vector<Request>::iterator it = messages_.begin();
    while (it != messages_.end() && !to_be_deleted)
    {
      if ((current_time - it->getStamp()) > expire_time_)
      {
        to_be_deleted = true;
      }
      else
      {
        it++;
      }
    }
    if (to_be_deleted)
    {
      // std::cout << "Erase (time expire): " << it->getRequester() << "\n";
      messages_.erase(it);
    }
    else
    {
      until_the_end = true;
    }
  }

  // If no request to merge return empty Request
  if (messages_.size() == 0)
  {
    // std::cout << " Merged 0 \n";
    return Request(requester_, current_time);
  }

  // If only one request return it
  if (messages_.size() == 1)
  {
    // std::cout << " Merged 1 \n";
    return messages_.at(0);
  }

  // If more than one request merge them
  // Sort requests in reverse order
  std::sort(messages_.begin(), messages_.end());
  std::reverse(messages_.begin(), messages_.end());

  // Combine request with lower priorities
  Request merged_req(messages_.at(0));
  // std::cout << " Merged " << messages_.size() << "\n";
  // std::cout << "--> " << messages_.at(0).getRequester() << "\n";
  for (unsigned int i = 1; i < messages_.size(); i++)
  {
    merged_req.combineRequest(messages_.at(i), type_);
    // std::cout << "--> " << messages_.at(i).getRequester() << "\n";
  }
  return merged_req;
}
