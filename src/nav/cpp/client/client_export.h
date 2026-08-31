#pragma once

#if defined(LINGTU_NAV_CLIENT_STATIC)
#  define LINGTU_NAV_CLIENT_API
#elif defined(_WIN32) || defined(__CYGWIN__)
#  if defined(LINGTU_NAV_CLIENT_BUILD)
#    define LINGTU_NAV_CLIENT_API __declspec(dllexport)
#  else
#    define LINGTU_NAV_CLIENT_API __declspec(dllimport)
#  endif
#else
#  if defined(LINGTU_NAV_CLIENT_BUILD)
#    define LINGTU_NAV_CLIENT_API __attribute__((visibility("default")))
#  else
#    define LINGTU_NAV_CLIENT_API
#  endif
#endif
