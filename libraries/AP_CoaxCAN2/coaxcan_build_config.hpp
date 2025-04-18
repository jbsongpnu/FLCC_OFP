/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef COAXCAN2_BUILD_CONFIG_HPP_INCLUDED
#define COAXCAN2_BUILD_CONFIG_HPP_INCLUDED

/**
 * COAXCAN version definition
 */
#define COAXCAN_VERSION_MAJOR    1
#define COAXCAN_VERSION_MINOR    0

/**
 * COAXCAN_CPP_VERSION - version of the C++ standard used during compilation.
 * This definition contains the integer year number after which the standard was named:
 *  - 2003 for C++03
 *  - 2011 for C++11
 *
 * This config automatically sets according to the actual C++ standard used by the compiler.
 *
 * In C++03 mode the library has almost zero dependency on the C++ standard library, which allows
 * to use it on platforms with a very limited C++ support. On the other hand, C++11 mode requires
 * many parts of the standard library (e.g. <functional>), thus the user might want to force older
 * standard than used by the compiler, in which case this symbol can be overridden manually via
 * compiler flags.
 */
#define COAXCAN_CPP11    2011
#define COAXCAN_CPP03    2003

#ifndef COAXCAN_CPP_VERSION
# if __cplusplus > 201200
#  error Unsupported C++ standard. You can explicitly set COAXCAN_CPP_VERSION=COAXCAN_CPP11 to silence this error.
# elif (__cplusplus > 201100) || defined(__GXX_EXPERIMENTAL_CXX0X__)
#  define COAXCAN_CPP_VERSION    COAXCAN_CPP11
# else
#  define COAXCAN_CPP_VERSION    COAXCAN_CPP03
# endif
#endif

/**
 * The library uses COAXCAN_NULLPTR instead of COAXCAN_NULLPTR and nullptr in order to allow the use of
 * -Wzero-as-null-pointer-constant.
 */
#ifndef COAXCAN_NULLPTR
# if COAXCAN_CPP_VERSION >= COAXCAN_CPP11
#  define COAXCAN_NULLPTR nullptr
# else
#  define COAXCAN_NULLPTR NULL
# endif
#endif

/**
 * By default, libuavcan enables all features if it detects that it is being built for a general-purpose
 * target like Linux. Value of this macro influences other configuration options located below in this file.
 * This macro can be overriden if needed.
 */
#ifndef COAXCAN_GENERAL_PURPOSE_PLATFORM
# if (defined(__linux__)    || defined(__linux)     || defined(__APPLE__)   ||\
      defined(_WIN64)       || defined(_WIN32)      || defined(__ANDROID__) ||\
      defined(_SYSTYPE_BSD) || defined(__FreeBSD__))
#  define COAXCAN_GENERAL_PURPOSE_PLATFORM 1
# else
#  define COAXCAN_GENERAL_PURPOSE_PLATFORM 0
# endif
#endif

/**
 * This macro enables built-in runtime checks and debug output via printf().
 * Should be used only for library development.
 */
#ifndef COAXCAN_DEBUG
# define COAXCAN_DEBUG 0
#endif

/**
 * This option allows to select whether libuavcan should throw exceptions on fatal errors, or try to handle
 * errors differently. By default, exceptions will be enabled only if the library is built for a general-purpose
 * OS like Linux. Set UAVCAN_EXCEPTIONS explicitly to override.
 */
#ifndef COAXCAN_EXCEPTIONS
# define COAXCAN_EXCEPTIONS COAXCAN_GENERAL_PURPOSE_PLATFORM
#endif

/**
 * This specification is used by some error reporting functions like in the Logger class.
 * The default can be overriden by defining the macro UAVCAN_NOEXCEPT explicitly, e.g. via compiler options.
 */
#ifndef COAXCAN_NOEXCEPT
# if COAXCAN_EXCEPTIONS
#  if COAXCAN_CPP_VERSION >= COAXCAN_CPP11
#   define COAXCAN_NOEXCEPT noexcept
#  else
#   define COAXCAN_NOEXCEPT throw()
#  endif
# else
#  define COAXCAN_NOEXCEPT
# endif
#endif

/**
 * Declaration visibility
 * http://gcc.gnu.org/wiki/Visibility
 */
#ifndef COAXCAN_EXPORT
# define COAXCAN_EXPORT
#endif

/**
 * Trade-off between ROM/RAM usage and functionality/determinism.
 * Note that this feature is not well tested and should be avoided.
 * Use code search for UAVCAN_TINY to find what functionality will be disabled.
 * This is particularly useful for embedded systems with less than 40kB of ROM.
 */
#ifndef COAXCAN_TINY
# define COAXCAN_TINY 0
#endif

/**
 * Disable the global data type registry, which can save some space on embedded systems.
 */
#ifndef COAXCAN_NO_GLOBAL_DATA_TYPE_REGISTRY
# define COAXCAN_NO_GLOBAL_DATA_TYPE_REGISTRY 0
#endif

/**
 * toString() methods will be disabled by default, unless the library is built for a general-purpose target like Linux.
 * It is not recommended to enable toString() on embedded targets as code size will explode.
 */
#ifndef COAXCAN_TOSTRING
# if COAXCAN_EXCEPTIONS
#  define COAXCAN_TOSTRING COAXCAN_GENERAL_PURPOSE_PLATFORM
# else
#  define COAXCAN_TOSTRING 0
# endif
#endif

#if COAXCAN_TOSTRING
# if !COAXCAN_EXCEPTIONS
#  error COAXCAN_TOSTRING requires COAXCAN_EXCEPTIONS
# endif
# include <string>
#endif

/**
 * Some C++ implementations are half-broken because they don't implement the placement new operator.
 * If UAVCAN_IMPLEMENT_PLACEMENT_NEW is defined, libuavcan will implement its own operator new (std::size_t, void*)
 * and its delete() counterpart, instead of relying on the standard header <new>.
 */
#ifndef COAXCAN_IMPLEMENT_PLACEMENT_NEW
# define COAXCAN_IMPLEMENT_PLACEMENT_NEW 0
#endif

/**
 * Allows the user's application to provide custom implementation of uavcan::snprintf(),
 * which is often useful on deeply embedded systems.
 */
#ifndef COAXCAN_USE_EXTERNAL_SNPRINTF
# define COAXCAN_USE_EXTERNAL_SNPRINTF   0
#endif

/**
 * Allows the user's application to provide a custom implementation of IEEE754Converter::nativeIeeeToHalf and
 * IEEE754Converter::halfToNativeIeee.
 */
#ifndef COAXCAN_USE_EXTERNAL_FLOAT16_CONVERSION
# define COAXCAN_USE_EXTERNAL_FLOAT16_CONVERSION 0
#endif

/**
 * Run time checks.
 * Resolves to the standard assert() by default.
 * Disabled completely if UAVCAN_NO_ASSERTIONS is defined.
 */
#define COAXCAN_NO_ASSERTIONS 1 //igpark

#ifndef COAXCAN_ASSERT
# ifndef COAXCAN_NO_ASSERTIONS
#  define COAXCAN_NO_ASSERTIONS 0
# endif
# if COAXCAN_NO_ASSERTIONS
#  define COAXCAN_ASSERT(x)
# else
#  define COAXCAN_ASSERT(x) assert(x)
# endif
#endif

#ifndef COAXCAN_LIKELY
# if __GNUC__
#  define COAXCAN_LIKELY(x) __builtin_expect(!!(x), true)
# else
#  define COAXCAN_LIKELY(x) (x)
# endif
#endif

#ifndef COAXCAN_UNLIKELY
# if __GNUC__
#  define COAXCAN_UNLIKELY(x) __builtin_expect(!!(x), false)
# else
#  define COAXCAN_UNLIKELY(x) (x)
# endif
#endif

namespace coaxcan2
{
/**
 * Memory pool block size.
 *
 * The default of 64 bytes should be OK for any target arch up to AMD64 and any compiler.
 *
 * The library leverages compile-time checks to ensure that all types that are subject to dynamic allocation
 * fit this size, otherwise compilation fails.
 *
 * For platforms featuring small pointer width (16..32 bits), UAVCAN_MEM_POOL_BLOCK_SIZE can often be safely
 * reduced to 56 or even 48 bytes, which leads to lower memory footprint.
 *
 * Note that the pool block size shall be aligned at biggest alignment of the target platform (detected and
 * checked automatically at compile time).
 */
#ifdef COAXCAN_MEM_POOL_BLOCK_SIZE
/// Explicitly specified by the user.
static const unsigned MemPoolBlockSize = COAXCAN_MEM_POOL_BLOCK_SIZE;
#elif defined(__BIGGEST_ALIGNMENT__) && (__BIGGEST_ALIGNMENT__ <= 8)
/// Convenient default for GCC-like compilers - if alignment allows, pool block size can be safely reduced.
static const unsigned MemPoolBlockSize = 56;
#else
/// Safe default that should be OK for any platform.
static const unsigned MemPoolBlockSize = 64;
#endif

#ifdef __BIGGEST_ALIGNMENT__
static const unsigned MemPoolAlignment = __BIGGEST_ALIGNMENT__;
#else
static const unsigned MemPoolAlignment = 16;
#endif

typedef char _alignment_check_for_MEM_POOL_BLOCK_SIZE[((MemPoolBlockSize & (MemPoolAlignment - 1)) == 0) ? 1 : -1];

/**
 * This class that allows to check at compile time whether type T can be allocated using the memory pool.
 * If the check fails, compilation fails.
 */
template <typename T>
struct COAXCAN_EXPORT IsDynamicallyAllocatable
{
    static void check()
    {
        char dummy[(sizeof(T) <= MemPoolBlockSize) ? 1 : -1] = { '0' };
        (void)dummy;
    }
};




}

#endif // COAXCAN2_BUILD_CONFIG_HPP_INCLUDED
