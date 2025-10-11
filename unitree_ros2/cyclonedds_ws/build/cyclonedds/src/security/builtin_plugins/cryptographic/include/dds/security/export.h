
#ifndef SECURITY_EXPORT_H
#define SECURITY_EXPORT_H

#ifdef SECURITY_STATIC_DEFINE
#  define SECURITY_EXPORT
#  define SECURITY_NO_EXPORT
#else
#  ifndef SECURITY_EXPORT
#    ifdef dds_security_crypto_EXPORTS
        /* We are building this library */
#      define SECURITY_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define SECURITY_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef SECURITY_NO_EXPORT
#    define SECURITY_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef SECURITY_DEPRECATED
#  define SECURITY_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef SECURITY_DEPRECATED_EXPORT
#  define SECURITY_DEPRECATED_EXPORT SECURITY_EXPORT SECURITY_DEPRECATED
#endif

#ifndef SECURITY_DEPRECATED_NO_EXPORT
#  define SECURITY_DEPRECATED_NO_EXPORT SECURITY_NO_EXPORT SECURITY_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef SECURITY_NO_DEPRECATED
#    define SECURITY_NO_DEPRECATED
#  endif
#endif

#ifndef SECURITY_INLINE_EXPORT
#  if __MINGW32__ && (!defined(__clang__) || !defined(dds_security_crypto_EXPORTS))
#    define SECURITY_INLINE_EXPORT
#  else
#    define SECURITY_INLINE_EXPORT SECURITY_EXPORT
#  endif
#endif

#endif /* SECURITY_EXPORT_H */
