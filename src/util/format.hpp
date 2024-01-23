#pragma once

// macOS does not have std::format yet
// So we define it and redirect it to fmt::format

#ifdef __APPLE__
#include <fmt/core.h>

namespace std {
    template<typename... T>
    FMT_NODISCARD FMT_INLINE auto format(fmt::format_string<T...> fmt, T&&... args) -> std::string {
        return fmt::format(fmt, std::forward<T>(args)...);
    }
} // namespace std
#else
#include <format>
#endif
