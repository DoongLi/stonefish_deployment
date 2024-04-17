/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib_ros/types.h"

#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <string>

namespace cola2
{
namespace ros
{
std::string cvTypeToImageEncoding(int cv_type)
{
  switch (cv_type)
  {
    case CV_8UC1:
      return sensor_msgs::image_encodings::TYPE_8UC1;
    case CV_8UC2:
      return sensor_msgs::image_encodings::TYPE_8UC2;
    case CV_8UC3:
      return sensor_msgs::image_encodings::TYPE_8UC3;
    case CV_8UC4:
      return sensor_msgs::image_encodings::TYPE_8UC4;
    case CV_8SC1:
      return sensor_msgs::image_encodings::TYPE_8SC1;
    case CV_8SC2:
      return sensor_msgs::image_encodings::TYPE_8SC2;
    case CV_8SC3:
      return sensor_msgs::image_encodings::TYPE_8SC3;
    case CV_8SC4:
      return sensor_msgs::image_encodings::TYPE_8SC4;
    case CV_16UC1:
      return sensor_msgs::image_encodings::TYPE_16UC1;
    case CV_16UC2:
      return sensor_msgs::image_encodings::TYPE_16UC2;
    case CV_16UC3:
      return sensor_msgs::image_encodings::TYPE_16UC3;
    case CV_16UC4:
      return sensor_msgs::image_encodings::TYPE_16UC4;
    case CV_16SC1:
      return sensor_msgs::image_encodings::TYPE_16SC1;
    case CV_16SC2:
      return sensor_msgs::image_encodings::TYPE_16SC2;
    case CV_16SC3:
      return sensor_msgs::image_encodings::TYPE_16SC3;
    case CV_16SC4:
      return sensor_msgs::image_encodings::TYPE_16SC4;
    case CV_32SC1:
      return sensor_msgs::image_encodings::TYPE_32SC1;
    case CV_32SC2:
      return sensor_msgs::image_encodings::TYPE_32SC2;
    case CV_32SC3:
      return sensor_msgs::image_encodings::TYPE_32SC3;
    case CV_32SC4:
      return sensor_msgs::image_encodings::TYPE_32SC4;
    case CV_32FC1:
      return sensor_msgs::image_encodings::TYPE_32FC1;
    case CV_32FC2:
      return sensor_msgs::image_encodings::TYPE_32FC2;
    case CV_32FC3:
      return sensor_msgs::image_encodings::TYPE_32FC3;
    case CV_32FC4:
      return sensor_msgs::image_encodings::TYPE_32FC4;
    case CV_64FC1:
      return sensor_msgs::image_encodings::TYPE_64FC1;
    case CV_64FC2:
      return sensor_msgs::image_encodings::TYPE_64FC2;
    case CV_64FC3:
      return sensor_msgs::image_encodings::TYPE_64FC3;
    case CV_64FC4:
      return sensor_msgs::image_encodings::TYPE_64FC4;
    default:
      throw std::runtime_error("Unknown CV type " + std::to_string(cv_type));
  }
}

int imageEncodingToCvType(const std::string &encoding)
{
  if (encoding == sensor_msgs::image_encodings::TYPE_8UC1 || encoding == sensor_msgs::image_encodings::MONO8 ||
      encoding == sensor_msgs::image_encodings::BAYER_RGGB8 || encoding == sensor_msgs::image_encodings::BAYER_BGGR8 ||
      encoding == sensor_msgs::image_encodings::BAYER_GBRG8 || encoding == sensor_msgs::image_encodings::BAYER_GRBG8)
  {
    return CV_8UC1;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_8UC2)
  {
    return CV_8UC2;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_8UC3 || encoding == sensor_msgs::image_encodings::RGB8 ||
           encoding == sensor_msgs::image_encodings::BGR8)
  {
    return CV_8UC3;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_8UC4 || encoding == sensor_msgs::image_encodings::RGBA8 ||
           encoding == sensor_msgs::image_encodings::BGRA8)
  {
    return CV_8UC4;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_8SC1)
  {
    return CV_8SC1;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_8SC2)
  {
    return CV_8SC2;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_8SC3)
  {
    return CV_8SC3;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_8SC4)
  {
    return CV_8SC4;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 || encoding == sensor_msgs::image_encodings::MONO16 ||
           encoding == sensor_msgs::image_encodings::BAYER_RGGB16 ||
           encoding == sensor_msgs::image_encodings::BAYER_BGGR16 ||
           encoding == sensor_msgs::image_encodings::BAYER_GBRG16 ||
           encoding == sensor_msgs::image_encodings::BAYER_GRBG16)
  {
    return CV_16UC1;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_16UC2)
  {
    return CV_16UC2;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_16UC3 || encoding == sensor_msgs::image_encodings::RGB16 ||
           encoding == sensor_msgs::image_encodings::BGR16)
  {
    return CV_16UC3;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_16UC4 || encoding == sensor_msgs::image_encodings::RGBA16 ||
           encoding == sensor_msgs::image_encodings::BGRA16)
  {
    return CV_16UC4;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_16SC1)
  {
    return CV_16SC1;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_16SC2)
  {
    return CV_16SC2;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_16SC3)
  {
    return CV_16SC3;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_16SC4)
  {
    return CV_16SC4;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32SC1)
  {
    return CV_32SC1;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32SC2)
  {
    return CV_32SC2;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32SC3)
  {
    return CV_32SC3;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32SC4)
  {
    return CV_32SC4;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    return CV_32FC1;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32FC2)
  {
    return CV_32FC2;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32FC3)
  {
    return CV_32FC3;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_32FC4)
  {
    return CV_32FC4;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_64FC1)
  {
    return CV_64FC1;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_64FC2)
  {
    return CV_64FC2;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_64FC3)
  {
    return CV_64FC3;
  }
  else if (encoding == sensor_msgs::image_encodings::TYPE_64FC4)
  {
    return CV_64FC4;
  }
  throw std::runtime_error("Unkown image encoding type " + encoding);
}

}  // namespace ros
}  // namespace cola2
