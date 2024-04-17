
/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Navigation helper for COLA2.
 */

#pragma once
#include <cola2_msgs/NavSts.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace cola2
{
namespace ros
{
/**
 * @addtogroup NavigationHelper
 * @{
 */

/**
 * @brief Determine if a navigation message is valid.
 *
 * @param msg Navigation message.
 * @return Validity of navigation message.
 */
bool navigationIsValid(const cola2_msgs::NavSts &msg);

/**
 * @brief Create a Invalid Navigation object.
 *
 * @return Invalid navigation.
 */
cola2_msgs::NavSts createInvalidNavigation();

/**
 * @brief Determine if a usbl message is valid.
 *
 * @param msg usbl message.
 * @return Validity of usbl message.
 */
bool usblIsValid(const geometry_msgs::PoseWithCovarianceStamped &msg);

/**
 * @brief Determine if a usbl message is valid.
 *
 * @param msg usbl message.
 * @param stamp current timestamp to check against.
 * @return Validity of usbl message.
 */
bool usblIsValid(const geometry_msgs::PoseWithCovarianceStamped &msg, const ::ros::Time &stamp);

/** @} */
}  // namespace ros
}  // namespace cola2
