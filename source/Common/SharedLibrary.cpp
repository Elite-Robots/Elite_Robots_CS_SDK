#include "SharedLibrary.hpp"
#include <Elite/Log.hpp>
#include <iostream>
#include <string>
#include <vector>

#if defined(__linux__)
#include <dlfcn.h>
#include <link.h>
#elif defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif  // platform

namespace ELITE {
SharedLibrary::SharedLibrary() : lib_handle_(nullptr) {}

SharedLibrary::~SharedLibrary() {}

bool SharedLibrary::loadLibrary(const std::string& library_path) {
    if (library_path.empty()) {
        return false;
    }
#if defined(__linux__)
    lib_handle_ = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (lib_handle_ == nullptr) {
        ELITE_LOG_ERROR("dlopen error: %s", dlerror());
        return false;
    }

    struct link_map* map = NULL;
    if (dlinfo(lib_handle_, RTLD_DI_LINKMAP, &map) != 0) {
        ELITE_LOG_ERROR("dlinfo error: %s", dlerror());
        goto load_fail;
    }
    lib_path_ = map->l_name;

    if (lib_path_.empty()) {
        ELITE_LOG_ERROR("Unable to get library path");
        goto load_fail;
    }

    return true;
load_fail:
    if (dlclose(lib_handle_) != 0) {
        ELITE_LOG_ERROR("dlclose error: %s\n", dlerror());
    }
    lib_handle_ = nullptr;
    return false;
#elif defined(_WIN32)
    lib_handle_ = LoadLibraryA(library_path.c_str());
    if (lib_handle_ == nullptr) {
        DWORD error_code = GetLastError();
        ELITE_LOG_ERROR("LoadLibraryA error: %lu", error_code);
        return false;
    }
    std::string path;
    std::vector<wchar_t> wpath(MAX_PATH, 0);
    DWORD chars_copied = 0;
    while (true) {
        chars_copied = GetModuleFileNameW(static_cast<HMODULE>(lib_handle_), wpath.data(), static_cast<DWORD>(wpath.size()));
        if (chars_copied == 0) {
            DWORD error_code = GetLastError();
            ELITE_LOG_ERROR("GetModuleFileNameW error: %lu", error_code);
            goto load_fail;
        }
        if (chars_copied < wpath.size()) {
            break;
        }
        wpath.resize(wpath.size() * 2);
    }

    int utf8_size = WideCharToMultiByte(CP_UTF8, 0, wpath.data(), static_cast<int>(chars_copied), nullptr, 0, nullptr, nullptr);
    if (utf8_size == 0) {
        DWORD error_code = GetLastError();
        ELITE_LOG_ERROR("WideCharToMultiByte error: %lu", error_code);
        goto load_fail;
    }

    path.resize(utf8_size, '\0');
    WideCharToMultiByte(CP_UTF8, 0, wpath.data(), static_cast<int>(chars_copied), &path[0], utf8_size, nullptr, nullptr);
    lib_path_ = path;

    if (lib_path_.empty()) {
        ELITE_LOG_ERROR("Unable to get library path");
        goto load_fail;
    }

    return true;
load_fail:
    if (!FreeLibrary(static_cast<HMODULE>(lib_handle_))) {
        DWORD error_code = GetLastError();
        ELITE_LOG_ERROR("FreeLibrary error: %lu", error_code);
    }
    lib_handle_ = nullptr;
    return false;

#else
#error "Unsupported platform"
#endif
}

bool SharedLibrary::unloadLibrary() {
    if (!lib_handle_) {
        return false;
    }
#if defined(__linux__)
    // The function dlclose() returns 0 on success, and nonzero on error.
    int error_code = dlclose(lib_handle_);
    if (error_code) {
        ELITE_LOG_ERROR("dlclose error: %s", dlerror());
        return false;
    }
    lib_handle_ = nullptr;
    lib_path_.clear();
#elif defined(_WIN32)
    if (!FreeLibrary(static_cast<HMODULE>(lib_handle_))) {
        DWORD error_code = GetLastError();
        ELITE_LOG_ERROR("FreeLibrary error: %lu", error_code);
        return false;
    }
    lib_handle_ = nullptr;
    lib_path_.clear();
#else
#error "Unsupported platform"
#endif
    return true;
}

void* SharedLibrary::getSymbol(const std::string& symbol_name) {
    if (!lib_handle_ || symbol_name.empty()) {
        ELITE_LOG_ERROR("Shared library get symbol has invalid inputs arguments");
        return nullptr;
    }

    void* lib_symbol = nullptr;
#if defined(__linux__)
    lib_symbol = dlsym(lib_handle_, symbol_name.c_str());
    char* error = dlerror();
    if (error != NULL) {
        ELITE_LOG_ERROR("Error getting the symbol '%s'. Error '%s'", symbol_name.c_str(), error);
        return nullptr;
    }
#elif defined(_WIN32)
    lib_symbol = GetProcAddress(static_cast<HMODULE>(lib_handle_), symbol_name.c_str());
    if (lib_symbol == nullptr) {
        DWORD error_code = GetLastError();
        ELITE_LOG_ERROR("Error getting the symbol '%s'. Error '%lu'", symbol_name.c_str(), error_code);
        return nullptr;
    }
#else
#error "Unsupported platform"
#endif
    if (!lib_symbol) {
        ELITE_LOG_ERROR("Symbol '%s' does not exist in the library '%s'", symbol_name.c_str(), lib_path_.c_str());
        return nullptr;
    }
    return lib_symbol;
}

bool SharedLibrary::hasSymbol(const std::string& symbol_name) {
    if (!lib_handle_ || symbol_name.empty()) {
        ELITE_LOG_ERROR("Shared library Querying for symbol existence returns an invalid parameter error.");
        return false;
    }
#if defined(__linux__)
    // The proper error-checking procedure is: first call dlerror() to clear any previous error state,
    // then call dlsym(), followed immediately by another dlerror() call, saving its return value
    // to a variable, and finally checking whether this stored value is not NULL.
    dlerror();
    void* lib_symbol = dlsym(lib_handle_, symbol_name.c_str());
    return dlerror() == NULL && lib_symbol != 0;
#elif defined(_WIN32)
    void* lib_symbol = GetProcAddress(static_cast<HMODULE>(lib_handle_), symbol_name.c_str());
    return lib_symbol != nullptr;
#else
#error "Unsupported platform"
#endif
}

std::string SharedLibrary::getLibraryPath() { return lib_path_; }

}  // namespace ELITE
