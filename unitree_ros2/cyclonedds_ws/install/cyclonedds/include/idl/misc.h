/*
 * Copyright(c) 2006 to 2021 ZettaScale Technology and others
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License v. 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
 * v. 1.0 which is available at
 * http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
 */
#ifndef IDL_MISC_H
#define IDL_MISC_H

#include <limits.h>

#if defined (__cplusplus)
extern "C" {
#endif

#define IDL_STRINGIFY(x) IDL_STRINGIFY1(x)
#define IDL_STRINGIFY1(x) #x

#if defined(__clang__) || \
    defined(__GNUC__) && ((__GNUC__ * 100) + __GNUC_MINOR__) >= 402
# define IDL_STR(s) #s
# define IDL_JOINSTR(x,y) IDL_STR(x ## y)
# define IDL_DO_PRAGMA(x) _Pragma(#x)
# define IDL_PRAGMA(x) IDL_DO_PRAGMA(GCC diagnostic x)

# if defined(__clang__)
#   define IDL_WARNING_CLANG_OFF(x) \
      IDL_PRAGMA(push) \
      IDL_PRAGMA(ignored IDL_JOINSTR(-W,x))
#   define IDL_WARNING_CLANG_ON(x) \
      IDL_PRAGMA(pop)
# elif ((__GNUC__ * 100) + __GNUC_MINOR__) >= 406
#   define IDL_WARNING_GNUC_OFF(x) \
      IDL_PRAGMA(push) \
      IDL_PRAGMA(ignored IDL_JOINSTR(-W,x))
#   define IDL_WARNING_GNUC_ON(x) \
      IDL_PRAGMA(pop)
# else
#   define IDL_WARNING_GNUC_OFF(x) \
      IDL_PRAGMA(ignored IDL_JOINSTR(-W,x))
#   define IDL_WARNING_GNUC_ON(x) \
      IDL_PRAGMA(warning IDL_JOINSTR(-W,x))
# endif
#endif

#if !defined(IDL_WARNING_CLANG_OFF) && \
    !defined(IDL_WARNING_CLANG_ON)
# define IDL_WARNING_CLANG_OFF(x)
# define IDL_WARNING_CLANG_ON(x)
#endif

#if !defined(IDL_WARNING_GNUC_OFF) && \
    !defined(IDL_WARNING_GNUC_ON)
# define IDL_WARNING_GNUC_OFF(x)
# define IDL_WARNING_GNUC_ON(x)
#endif

#if defined(_MSC_VER)
# define IDL_WARNING_MSVC_OFF(x) \
    __pragma (warning(push)) \
    __pragma (warning(disable: ## x))
# define IDL_WARNING_MSVC_ON(x) \
    __pragma (warning(pop))
#else
# define IDL_WARNING_MSVC_OFF(x)
# define IDL_WARNING_MSVC_ON(x)
#endif

/**
 * @brief Macro to disable unused argument warnings
 */
#define IDL_UNUSED_ARG(a) (void)(a)

/**
 * @brief Macro to disable warnings for calling deprecated interfaces
 */
#define IDL_WARNING_DEPRECATED_OFF \
  IDL_WARNING_CLANG_OFF(deprecated-declarations) \
  IDL_WARNING_GNUC_OFF(deprecated-declarations) \
  IDL_WARNING_MSVC_OFF(4996)

/**
 * @brief Macro to enable warnings for calling deprecated interfaces
 */
#define IDL_WARNING_DEPRECATED_ON \
  IDL_WARNING_CLANG_ON(deprecated-declarations) \
  IDL_WARNING_GNUC_ON(deprecated-declarations) \
  IDL_WARNING_MSVC_ON(4996)

#if defined (__cplusplus)
}
#endif

#endif /* IDL_MISC_H */
