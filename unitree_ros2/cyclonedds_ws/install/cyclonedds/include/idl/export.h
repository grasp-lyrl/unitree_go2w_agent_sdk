
#ifndef IDL_EXPORT_H
#define IDL_EXPORT_H

#ifdef IDL_STATIC_DEFINE
#  define IDL_EXPORT
#  define IDL_NO_EXPORT
#else
#  ifndef IDL_EXPORT
#    ifdef idl_EXPORTS
        /* We are building this library */
#      define IDL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define IDL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef IDL_NO_EXPORT
#    define IDL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef IDL_DEPRECATED
#  define IDL_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef IDL_DEPRECATED_EXPORT
#  define IDL_DEPRECATED_EXPORT IDL_EXPORT IDL_DEPRECATED
#endif

#ifndef IDL_DEPRECATED_NO_EXPORT
#  define IDL_DEPRECATED_NO_EXPORT IDL_NO_EXPORT IDL_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef IDL_NO_DEPRECATED
#    define IDL_NO_DEPRECATED
#  endif
#endif

#ifndef IDL_INLINE_EXPORT
#  if __MINGW32__ && (!defined(__clang__) || !defined(idl_EXPORTS))
#    define IDL_INLINE_EXPORT
#  else
#    define IDL_INLINE_EXPORT IDL_EXPORT
#  endif
#endif

#endif /* IDL_EXPORT_H */
