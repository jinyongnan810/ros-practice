#ifndef ACTION_CPP_PKG__VISIBILITY_CONTROL_H_
#define ACTION_CPP_PKG__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_CPP_PKG_EXPORT __attribute__ ((dllexport))
    #define ACTION_CPP_PKG_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_CPP_PKG_EXPORT __declspec(dllexport)
    #define ACTION_CPP_PKG_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_CPP_PKG_BUILDING_DLL
    #define ACTION_CPP_PKG_PUBLIC ACTION_CPP_PKG_EXPORT
  #else
    #define ACTION_CPP_PKG_PUBLIC ACTION_CPP_PKG_IMPORT
  #endif
  #define ACTION_CPP_PKG_PUBLIC_TYPE ACTION_CPP_PKG_PUBLIC
  #define ACTION_CPP_PKG_LOCAL
#else
  #define ACTION_CPP_PKG_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_CPP_PKG_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_CPP_PKG_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_CPP_PKG_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_CPP_PKG_PUBLIC
    #define ACTION_CPP_PKG_LOCAL
  #endif
  #define ACTION_CPP_PKG_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_CPP_PKG__VISIBILITY_CONTROL_H_
