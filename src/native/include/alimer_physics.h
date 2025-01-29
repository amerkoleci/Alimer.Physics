// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

#ifndef _ALIMER_PHYSICS_H
#define _ALIMER_PHYSICS_H

#if defined(ALIMER_SHARED_LIBRARY)
#    if defined(_WIN32)
#        if defined(ALIMER_IMPLEMENTATION)
#            define _ALIMER_EXPORT __declspec(dllexport)
#        else
#            define _ALIMER_EXPORT __declspec(dllimport)
#        endif
#    else
#        if defined(ALIMER_IMPLEMENTATION)
#            define _ALIMER_EXPORT __attribute__((visibility("default")))
#        else
#            define _ALIMER_EXPORT
#        endif
#    endif
#else
#    define _ALIMER_EXPORT
#endif

#ifdef __cplusplus
#    define _ALIMER_EXTERN extern "C"
#else
#    define _ALIMER_EXTERN extern
#endif

#define ALIMER_API _ALIMER_EXTERN _ALIMER_EXPORT 

#ifdef _WIN32
#   define ALIMER_CALL __cdecl
#else
#   define ALIMER_CALL
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum LogLevel {
    LogLevel_Off = 0,
    LogLevel_Trace = 1,
    LogLevel_Debug = 2,
    LogLevel_Info = 3,
    LogLevel_Warn = 4,
    LogLevel_Error = 5,
    LogLevel_Fatal = 6,

    LogLevel_Count,
    _LogLevel_Force32 = 0x7FFFFFFF
} LogLevel;

typedef struct PhysicsConfig {
    void* userdata;
    void (*logCallback)(void* userdata, const char* message);
    void* (*allocCallback)(size_t size);
    void (*freeCallback)(void* data);

    uint32_t tempAllocatorInitSize;
    uint32_t maxPhysicsJobs;
    uint32_t maxPhysicsBarriers;
} PhysicsConfig;

ALIMER_API bool alimerPhysicsInit(const PhysicsConfig* config);
ALIMER_API void alimerPhysicsShutdown(void);

#endif /* _ALIMER_PHYSICS_H */
