// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.

#include "ClassRegistry.hpp"
#include <string>
#include "Log.hpp"

namespace ELITE {

namespace INTERNAL {

ClassRegistry& ClassRegistry::instance() {
    static ClassRegistry inst;
    return inst;
}

ClassRegistry::ClassRegistry() {}

bool ClassRegistry::registerClass(const std::string& derived, const std::string& base, Factory factory) {
    return factories_.emplace(makeClassKey(derived, base), factory).second;
}

ClassRegistry::Factory ClassRegistry::getFactory(const std::string& derived, const std::string& base) const {
    auto it = factories_.find(makeClassKey(derived, base));
    if (it == factories_.end()) {
        return nullptr;
    }
    return it->second;
}

// Compose a unique key from derived and base class names.
// Uses a control character as a separator to minimize collision risk.
std::string ClassRegistry::makeClassKey(const std::string& derived, const std::string& base) const {
    static const char separator = '\x1F';  // Unit Separator
    return derived + separator + base;
}

}  // namespace INTERNAL

}  // namespace ELITE
