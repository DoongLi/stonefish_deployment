/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains functions to manipulate types.
 */

#pragma once

#include <string>

namespace cola2
{
namespace ros
{
/**
 * @addtogroup Types
 * @{
 */

/**
 * @brief Convert cv type to image encoding type (ROS)
 *
 * @param cv_type
 * @return
 * @throw Runtime error if the conversion was not possible.
 */
std::string cvTypeToImageEncoding(int cv_type);

/**
 * @brief Convert image encoding type (ROS) to cv type
 *
 * @param encoding
 * @return
 * @throw Runtime error if the conversion was not possible.
 */
int imageEncodingToCvType(const std::string &encoding);

/** @} */
}  // namespace ros
}  // namespace cola2
