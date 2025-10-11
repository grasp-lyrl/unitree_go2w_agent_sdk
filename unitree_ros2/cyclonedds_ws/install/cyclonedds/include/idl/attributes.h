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
#ifndef IDL_ATTRIBUTES_H
#define IDL_ATTRIBUTES_H

#if __GNUC__
# define idl_gnuc (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#else
# define idl_gnuc (0)
#endif

#if __clang__
# define idl_clang (__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__)
#else
# define idl_clang (0)
#endif

#ifdef __SUNPRO_C
# define __attribute__(x)
#endif

#if defined(__has_attribute)
# define idl_has_attribute(params) __has_attribute(params)
#elif idl_gnuc
# define idl_has_attribute(params) (1) /* GCC < 5 */
#else
# define idl_has_attribute(params) (0)
#endif

#if idl_has_attribute(malloc)
# define idl_attribute_malloc __attribute__ ((__malloc__))
#else
# define idl_attribute_malloc
#endif

#if idl_has_attribute(unused)
# define idl_attribute_unused __attribute__((__unused__))
#else
# define idl_attribute_unused
#endif

#if idl_has_attribute(noreturn)
# define idl_attribute_noreturn __attribute__ ((__noreturn__))
#else
# define idl_attribute_noreturn
#endif

#if idl_has_attribute(nonnull)
# define idl_nonnull(params) __attribute__ ((__nonnull__ params))
# define idl_nonnull_all __attribute__ ((__nonnull__))
#else
# define idl_nonnull(params)
# define idl_nonnull_all
#endif

#if idl_has_attribute(returns_nonnull) && (idl_clang || idl_gnuc >= 40900)
# define idl_attribute_returns_nonnull __attribute__ ((__returns_nonnull__))
#else
# define idl_attribute_returns_nonnull
#endif

/* GCC <= 4.2.4 has the attribute, but warns that it ignores it. */
#if !idl_has_attribute(alloc_size) || (idl_gnuc <= 40204)
# define idl_attribute_alloc_size(params)
#else
# define idl_attribute_alloc_size(params) __attribute__ ((__alloc_size__ params))
#endif

#if idl_has_attribute(const)
# define idl_attribute_const __attribute__ ((__const__))
#else
# define idl_attribute_const
#endif

#if idl_has_attribute(pure)
# define idl_attribute_pure __attribute__ ((__pure__))
#else
# define idl_attribute_pure
#endif

#if idl_has_attribute(format)
# define idl_attribute_format(params) __attribute__ ((__format__ params))
# if __MINGW32__
#   if !defined(__MINGW_PRINTF_FORMAT)
#     define __MINGW_PRINTF_FORMAT gnu_printf
#   endif
    /* GCC assumes printf MS style arguments on Windows */
#   define idl_attribute_format_printf(string_index, first_to_check) \
      idl_attribute_format((__MINGW_PRINTF_FORMAT, string_index, first_to_check))
# else
#   define idl_attribute_format_printf(string_index, first_to_check) \
      idl_attribute_format((printf, string_index, first_to_check))
# endif
#else
# define idl_attribute_format(params)
# define idl_attribute_format_printf(string_index, first_to_check)
#endif

#if idl_has_attribute(warn_unused_result)
# define idl_attribute_warn_unused_result __attribute__ ((__warn_unused_result__))
#else
# define idl_attribute_warn_unused_result
#endif

#if idl_has_attribute(assume_aligned)
# define idl_attribute_assume_aligned(params) __attribute__ ((__assume_aligned__ params))
#else
# define idl_attribute_assume_aligned(params)
#endif

#if idl_has_attribute(packed)
# define idl_attribute_packed __attribute__ ((__packed__))
#else
# define idl_attribute_packed
#endif

#if idl_has_attribute(no_sanitize)
# define idl_attribute_no_sanitize(params) __attribute__ ((__no_sanitize__ params))
#else
# define idl_attribute_no_sanitize(params)
#endif

#if defined(__has_feature)
# define idl_has_feature_thread_sanitizer __has_feature(thread_sanitizer)
#else
# define idl_has_feature_thread_sanitizer 0
#endif

#endif /* IDL_ATTRIBUTES_H */
