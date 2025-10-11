
#ifndef MCPP_EXPORT_H
#define MCPP_EXPORT_H

#ifdef MCPP_STATIC_DEFINE
#  define MCPP_EXPORT
#  define MCPP_NO_EXPORT
#else
#  ifndef MCPP_EXPORT
#    ifdef idlpp_EXPORTS
        /* We are building this library */
#      define MCPP_EXPORT 
#    else
        /* We are using this library */
#      define MCPP_EXPORT 
#    endif
#  endif

#  ifndef MCPP_NO_EXPORT
#    define MCPP_NO_EXPORT 
#  endif
#endif

#ifndef MCPP_DEPRECATED
#  define MCPP_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef MCPP_DEPRECATED_EXPORT
#  define MCPP_DEPRECATED_EXPORT MCPP_EXPORT MCPP_DEPRECATED
#endif

#ifndef MCPP_DEPRECATED_NO_EXPORT
#  define MCPP_DEPRECATED_NO_EXPORT MCPP_NO_EXPORT MCPP_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef MCPP_NO_DEPRECATED
#    define MCPP_NO_DEPRECATED
#  endif
#endif

#ifndef MCPP_INLINE_EXPORT
#  if __MINGW32__ && (!defined(__clang__) || !defined(idlpp_EXPORTS))
#    define MCPP_INLINE_EXPORT
#  else
#    define MCPP_INLINE_EXPORT MCPP_EXPORT
#  endif
#endif

#endif /* MCPP_EXPORT_H */
