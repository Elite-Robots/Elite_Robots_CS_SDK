#include "kinematics_base.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace ELITE {

KinematicsBasePtr loadKinematicsSolver(const std::string &library_path, const std::string &symbol_name) {
#ifdef _WIN32
    HMODULE handle = ::LoadLibraryA(library_path.c_str());
    if (!handle) return nullptr;

    auto fn = reinterpret_cast<CreateKinematicsFn>(::GetProcAddress(handle, symbol_name.c_str()));
    if (!fn) {
        ::FreeLibrary(handle);
        return nullptr;
    }
#else
    void *handle = ::dlopen(library_path.c_str(), RTLD_LAZY);
    if (!handle) return nullptr;

    auto fn = reinterpret_cast<CreateKinematicsFn>(::dlsym(handle, symbol_name.c_str()));
    if (!fn) {
        ::dlclose(handle);
        return nullptr;
    }
#endif

    KinematicsBase *raw = fn();
    if (!raw) {
#ifdef _WIN32
        ::FreeLibrary(handle);
#else
        ::dlclose(handle);
#endif
        return nullptr;
    }
    return KinematicsBasePtr(raw);
}

}  // namespace ELITE
