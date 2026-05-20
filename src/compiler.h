#pragma once

/// @file compiler.h
/// @brief Compiler hints and platform-specific macros (private to esp-audio-libs).
///
/// Wraps GCC/Clang builtins behind macros that fall back to portable C++ on other
/// compilers, so source files can use one spelling regardless of toolchain.

// Inline memcpy that bypasses ESP-IDF's -fno-builtin-memcpy. The plain `memcpy`
// name is forced to a real call so the linker can resolve it to the ROM-resident
// implementation; using `__builtin_memcpy` directly opts back into compile-time
// expansion, which is what we want for the small constant-size copies we use to
// dodge strict-aliasing UB without paying for a function call.
#if defined(__GNUC__) || defined(__clang__)
#define EAL_MEMCPY(dst, src, n) __builtin_memcpy((dst), (src), (n))
#else
#include <cstring>
#define EAL_MEMCPY(dst, src, n) std::memcpy((dst), (src), (n))
#endif

// Mark a function as hot: optimize it aggressively and group it with other hot code for better
// instruction-cache locality. No-op on compilers without the attribute.
#if defined(__GNUC__) || defined(__clang__)
#define EAL_HOT __attribute__((hot))
#else
#define EAL_HOT
#endif

// Tell the optimizer that `ptr` is aligned to `n` bytes. Caller is responsible
// for guaranteeing the precondition (typically via a runtime check). Lets
// follow-up loads/stores fold into single aligned-access instructions on
// architectures that distinguish them (e.g. Xtensa l16si/l32i vs byte-wise).
#if defined(__GNUC__) || defined(__clang__)
#define EAL_ASSUME_ALIGNED(ptr, n) __builtin_assume_aligned((ptr), (n))
#else
#define EAL_ASSUME_ALIGNED(ptr, n) (ptr)
#endif
