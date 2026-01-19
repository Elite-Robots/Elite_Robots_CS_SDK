// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// StringUtils.hpp
// Provides utility functions for string manipulation.
#ifndef __ELITE__STRING_UTILS_HPP__
#define __ELITE__STRING_UTILS_HPP__

#include <Elite/EliteOptions.hpp>

#include <string>
#include <vector>

namespace ELITE {

class StringUtils {
   private:
   public:
    StringUtils() = default;
    virtual ~StringUtils() = default;

    /**
     * @brief Split the string by the given delimiter.
     *
     * @param input
     * @param delimiter
     * @return std::vector<std::string> string list
     */
    ELITE_EXPORT static std::vector<std::string> splitString(const std::string& input, const std::string& delimiter);
};

}  // namespace ELITE

#endif