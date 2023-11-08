/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef PMUCAN_BUILD_CONFIG_HPP_INCLUDED
#define PMUCAN_BUILD_CONFIG_HPP_INCLUDED

/**
 * PMUCAN version definition
 */
#define PMUCAN_VERSION_MAJOR    1
#define PMUCAN_VERSION_MINOR    0

/**
 * PMUCAN_CPP_VERSION - version of the C++ standard used during compilation.
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
#define PMUCAN_CPP11    2011
#define PMUCAN_CPP03    2003

#ifndef PMUCAN_CPP_VERSION
# if __cplusplus > 201200
#  error Unsupported C++ standard. You can explicitly set PMUCAN_CPP_VERSION=PMUCAN_CPP11 to silence this error.
# elif (__cplusplus > 201100) || defined(__GXX_EXPERIMENTAL_CXX0X__)
#  define PMUCAN_CPP_VERSION    PMUCAN_CPP11
# else
#  define PMUCAN_CPP_VERSION    PMUCAN_CPP03
# endif
#endif

/**
 * The library uses PMUCAN_NULLPTR instead of PMUCAN_NULLPTR and nullptr in order to allow the use of
 * -Wzero-as-null-pointer-constant.
 */
#ifndef PMUCAN_NULLPTR
# if PMUCAN_CPP_VERSION >= PMUCAN_CPP11
#  define PMUCAN_NULLPTR nullptr
# else
#  define PMUCAN_NULLPTR NULL
# endif
#endif

/**
 * By default, libuavcan enables all features if it detects that it is being built for a general-purpose
 * target like Linux. Value of this macro influences other configuration options located below in this file.
 * This macro can be overriden if needed.
 */
#ifndef PMUCAN_GENERAL_PURPOSE_PLATFORM
# if (defined(__linux__)    || defined(__linux)     || defined(__APPLE__)   ||\
      defined(_WIN64)       || defined(_WIN32)      || defined(__ANDROID__) ||\
      defined(_SYSTYPE_BSD) || defined(__FreeBSD__))
#  define PMUCAN_GENERAL_PURPOSE_PLATFORM 1
# else
#  define PMUCAN_GENERAL_PURPOSE_PLATFORM 0
# endif
#endif

/**
 * This macro enables built-in runtime checks and debug output via printf().
 * Should be used only for library development.
 */
#ifndef PMUCAN_DEBUG
# define PMUCAN_DEBUG 0
#endif

/**
 * This option allows to select whether libuavcan should throw exceptions on fatal errors, or try to handle
 * errors differently. By default, exceptions will be enabled only if the library is built for a general-purpose
 * OS like Linux. Set UAVCAN_EXCEPTIONS explicitly to override.
 */
#ifndef PMUCAN_EXCEPTIONS
# define PMUCAN_EXCEPTIONS PMUCAN_GENERAL_PURPOSE_PLATFORM
#endif

/**
 * This specification is used by some error reporting functions like in the Logger class.
 * The default can be overriden by defining the macro UAVCAN_NOEXCEPT explicitly, e.g. via compiler options.
 */
#ifndef PMUCAN_NOEXCEPT
# if PMUCAN_EXCEPTIONS
#  if PMUCAN_CPP_VERSION >= PMUCAN_CPP11
#   define PMUCAN_NOEXCEPT noexcept
#  else
#   define PMUCAN_NOEXCEPT throw()
#  endif
# else
#  define PMUCAN_NOEXCEPT
# endif
#endif

/**
 * Declaration visibility
 * http://gcc.gnu.org/wiki/Visibility
 */
#ifndef PMUCAN_EXPORT
# define PMUCAN_EXPORT
#endif

/**
 * Trade-off between ROM/RAM usage and functionality/determinism.
 * Note that this feature is not well tested and should be avoided.
 * Use code search for UAVCAN_TINY to find what functionality will be disabled.
 * This is particularly useful for embedded systems with less than 40kB of ROM.
 */
#ifndef PMUCAN_TINY
# define PMUCAN_TINY 0
#endif

/**
 * Disable the global data type registry, which can save some space on embedded systems.
 */
#ifndef PMUCAN_NO_GLOBAL_DATA_TYPE_REGISTRY
# define PMUCAN_NO_GLOBAL_DATA_TYPE_REGISTRY 0
#endif

/**
 * toString() methods will be disabled by default, unless the library is built for a general-purpose target like Linux.
 * It is not recommended to enable toString() on embedded targets as code size will explode.
 */
#ifndef PMUCAN_TOSTRING
# if PMUCAN_EXCEPTIONS
#  define PMUCAN_TOSTRING PMUCAN_GENERAL_PURPOSE_PLATFORM
# else
#  define PMUCAN_TOSTRING 0
# endif
#endif

#if PMUCAN_TOSTRING
# if !PMUCAN_EXCEPTIONS
#  error PMUCAN_TOSTRING requires PMUCAN_EXCEPTIONS
# endif
# include <string>
#endif

/**
 * Some C++ implementations are half-broken because they don't implement the placement new operator.
 * If UAVCAN_IMPLEMENT_PLACEMENT_NEW is defined, libuavcan will implement its own operator new (std::size_t, void*)
 * and its delete() counterpart, instead of relying on the standard header <new>.
 */
#ifndef PMUCAN_IMPLEMENT_PLACEMENT_NEW
# define PMUCAN_IMPLEMENT_PLACEMENT_NEW 0
#endif

/**
 * Allows the user's application to provide custom implementation of uavcan::snprintf(),
 * which is often useful on deeply embedded systems.
 */
#ifndef PMUCAN_USE_EXTERNAL_SNPRINTF
# define PMUCAN_USE_EXTERNAL_SNPRINTF   0
#endif

/**
 * Allows the user's application to provide a custom implementation of IEEE754Converter::nativeIeeeToHalf and
 * IEEE754Converter::halfToNativeIeee.
 */
#ifndef PMUCAN_USE_EXTERNAL_FLOAT16_CONVERSION
# define PMUCAN_USE_EXTERNAL_FLOAT16_CONVERSION 0
#endif

/**
 * Run time checks.
 * Resolves to the standard assert() by default.
 * Disabled completely if UAVCAN_NO_ASSERTIONS is defined.
 */
#define PMUCAN_NO_ASSERTIONS 1 //igpark

#ifndef PMUCAN_ASSERT
# ifndef PMUCAN_NO_ASSERTIONS
#  define PMUCAN_NO_ASSERTIONS 0
# endif
# if PMUCAN_NO_ASSERTIONS
#  define PMUCAN_ASSERT(x)
# else
#  define PMUCAN_ASSERT(x) assert(x)
# endif
#endif

#ifndef PMUCAN_LIKELY
# if __GNUC__
#  define PMUCAN_LIKELY(x) __builtin_expect(!!(x), true)
# else
#  define PMUCAN_LIKELY(x) (x)
# endif
#endif

#ifndef PMUCAN_UNLIKELY
# if __GNUC__
#  define PMUCAN_UNLIKELY(x) __builtin_expect(!!(x), false)
# else
#  define PMUCAN_UNLIKELY(x) (x)
# endif
#endif

namespace pmucan
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
#ifdef PMUCAN_MEM_POOL_BLOCK_SIZE
/// Explicitly specified by the user.
static const unsigned MemPoolBlockSize = PMUCAN_MEM_POOL_BLOCK_SIZE;
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
struct PMUCAN_EXPORT IsDynamicallyAllocatable
{
    static void check()
    {
        char dummy[(sizeof(T) <= MemPoolBlockSize) ? 1 : -1] = { '0' };
        (void)dummy;
    }
};




}

#endif // PMUCAN_BUILD_CONFIG_HPP_INCLUDED
