/*
 * Debug stuff, should only be used for library development.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef PMUCAN_DEBUG_HPP_INCLUDED
#define PMUCAN_DEBUG_HPP_INCLUDED

#include "pmucan_build_config.hpp"

#if PMUCAN_DEBUG

# include <cstdio>
# include <cstdarg>

# if __GNUC__
__attribute__ ((format(printf, 2, 3)))
# endif
static void PMUCAN_TRACE(const char* src, const char* fmt, ...)
{
    va_list args;
    (void)std::printf("PMUCAN: %s: ", src);
    va_start(args, fmt);
    (void)std::vprintf(fmt, args);
    va_end(args);
    (void)std::puts("");
}

#else

# define PMUCAN_TRACE(...) ((void)0)

#endif

#endif // PMUCAN_DEBUG_HPP_INCLUDED
