/**
 * @file VersionInfo.hpp
 * @author yanxiaojia
 * @brief Containing a version information
 * @date 2024-08-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __VERSION_INFO_HPP__
#define __VERSION_INFO_HPP__

#include <string>

#include <Elite/EliteOptions.hpp>

namespace ELITE
{

// In some compilers, there may be macro definitions for major and minor.
#ifdef major
#undef major
#endif
#ifdef minor
#undef minor
#endif

/**
 * @brief Struct containing a Elite version information
 * 
 */
class VersionInfo {
public:
    /**
     * @brief Construct a constant for version information.
     * 
     * @param ma major
     * @param mi minor
     * @param bug bugfix
     * @param bui build
     */
    constexpr VersionInfo(int ma, int mi, int bug, int bui) : 
       major(ma), minor(mi), bugfix(bug), build(bui) { };
    
    /**
     * @brief Construct a new Version Info object
     * 
     * @param version The format string of version infomation
     */
    ELITE_EXPORT explicit VersionInfo(const std::string& version);

    VersionInfo() = default;

    ~VersionInfo() = default;

    /**
     * @brief Convert version information to a string.
     * 
     * @return std::string The string of version: major.minor.bugfix.build
     */
    ELITE_EXPORT std::string toString() const;

    /**
     * @brief Parses a version string into a VersionInformation object
     * 
     * @param str Version string
     * @return VersionInfo Version information
     */
    ELITE_EXPORT static VersionInfo fromString(const std::string& str);

    ELITE_EXPORT VersionInfo& operator=(const VersionInfo&);
    ELITE_EXPORT bool operator==(const VersionInfo& v) const;
    ELITE_EXPORT bool operator!=(const VersionInfo& v) const;
    ELITE_EXPORT bool operator>(const VersionInfo& v) const;
    ELITE_EXPORT bool operator>=(const VersionInfo& v) const;
    ELITE_EXPORT bool operator<(const VersionInfo& v) const;
    ELITE_EXPORT bool operator<=(const VersionInfo& v) const;

    ELITE_EXPORT constexpr bool operator==(VersionInfo& v) const;
    ELITE_EXPORT constexpr bool operator!=(VersionInfo& v) const;
    ELITE_EXPORT constexpr bool operator>(VersionInfo& v) const;
    ELITE_EXPORT constexpr bool operator>=(VersionInfo& v) const;
    ELITE_EXPORT constexpr bool operator<(VersionInfo& v) const;
    ELITE_EXPORT constexpr bool operator<=(VersionInfo& v) const;

    uint32_t major = 0;
    uint32_t minor = 0;
    uint32_t bugfix = 0;
    uint32_t build = 0;
};

/**
 * @brief Get the SDK version info
 */
constexpr VersionInfo SDK_VERSION_INFO(ELITE_SDK_VERSION_MAJOR, ELITE_SDK_VERSION_MINOR, ELITE_SDK_VERSION_BUGFIX, 0);

} // namespace ELITE

#endif
