#include "nuscenes2bag/utils.hpp"

#include <iostream>

namespace nuscenes2bag {

std::string
toLower(const std::string_view& str)
{
  std::string lowerStr;
  std::transform(
    str.begin(), str.end(), std::back_inserter(lowerStr), ::tolower);
  return lowerStr;
}

bool
string_icontains(const std::string_view& string, const std::string_view& sub)
{
  std::string lowerString = toLower(string);
  std::string lowerSub = toLower(sub);
  return lowerString.find(lowerSub) != std::string::npos;
}

ros::Time
stampUs2RosTime(uint64_t stampUs)
{
  ros::Time t;
  t = t.fromNSec(stampUs * 1000);
  return t;
}

}