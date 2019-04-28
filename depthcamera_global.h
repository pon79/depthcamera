#ifndef DEPTHCAMERA_GLOBAL_H
#define DEPTHCAMERA_GLOBAL_H

#if defined(_WIN32)
#   if defined(DEPTHCAMERA_LIBRARY)
#       define DEPTHCAMERASHARED_EXPORT __declspec(dllexport)
#   else
#       define DEPTHCAMERASHARED_EXPORT __declspec(dllimport)
#   endif
#elif __linux__
#   define DEPTHCAMERASHARED_EXPORT
#endif

#endif // DEPTHCAMERA_GLOBAL_H
