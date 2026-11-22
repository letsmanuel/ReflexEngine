#pragma once
/// @file export.h
/// Symbol visibility macro so ReflexEngine can be built as static or shared library.

#if defined(_WIN32) && defined(REFLEX_SHARED)
  #ifdef REFLEX_BUILDING
    #define REFLEX_API __declspec(dllexport)
  #else
    #define REFLEX_API __declspec(dllimport)
  #endif
  #pragma warning(disable : 4251) // STL members in exported classes are never touched by clients
#elif defined(REFLEX_SHARED) && (defined(__GNUC__) || defined(__clang__))
  #define REFLEX_API __attribute__((visibility("default")))
#else
  #define REFLEX_API
#endif
