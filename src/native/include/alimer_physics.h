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

/* Forward */
typedef struct PhysicsWorldImpl* PhysicsWorld;
typedef struct PhysicsBodyImpl* PhysicsBody;
typedef struct PhysicsShapeImpl* PhysicsShape;
typedef struct PhysicsMaterialImpl* PhysicsMaterial;

/* Enums */
typedef enum LogLevel {
    LogLevel_Error = 0,
    LogLevel_Warn = 1,
    LogLevel_Info = 2,
    LogLevel_Trace = 4,

    LogLevel_Count,
    _LogLevel_Force32 = 0x7FFFFFFF
} LogLevel;

typedef enum PhysicsShapeType {
    PhysicsShapeType_Box,
    PhysicsShapeType_Sphere,
    PhysicsShapeType_Capsule,
    PhysicsShapeType_Cylindex,
    PhysicsShapeType_Convex,
    PhysicsShapeType_Mesh,
    PhysicsShapeType_Terrain,

    _PhysicsShapeType_Count,
    _PhysicsShapeType_Force32 = 0x7FFFFFFF
} PhysicsShapeType;

typedef struct PhysicsConfig {
    void* userdata;
    void (*logCallback)(void* userdata, LogLevel level, const char* message);

    uint32_t tempAllocatorInitSize;
    uint32_t maxPhysicsJobs;
    uint32_t maxPhysicsBarriers;
} PhysicsConfig;

typedef struct PhysicsWorldConfig {
    uint32_t maxBodies;
    uint32_t maxBodyPairs;
} PhysicsWorldConfig;

ALIMER_API bool alimerPhysicsInit(const PhysicsConfig* config);
ALIMER_API void alimerPhysicsShutdown(void);

/* World */
ALIMER_API PhysicsWorld alimerPhysicsWorldCreate(const PhysicsWorldConfig* config);
ALIMER_API void alimerPhysicsWorldDestroy(PhysicsWorld world);
ALIMER_API uint32_t alimerPhysicsWorldGetBodyCount(PhysicsWorld world);
ALIMER_API uint32_t alimerPhysicsWorldGetActiveBodyCount(PhysicsWorld world);
ALIMER_API void alimerPhysicsWorldGetGravity(PhysicsWorld world, float gravity[3]);
ALIMER_API void alimerPhysicsWorldSetGravity(PhysicsWorld world, const float gravity[3]);
ALIMER_API bool alimerPhysicsWorldUpdate(PhysicsWorld world, float deltaTime, int collisionSteps);

/* Shape */
ALIMER_API void alimerPhysicsShapeAddRef(PhysicsShape shape);
ALIMER_API void alimerPhysicsShapeRelease(PhysicsShape shape);
ALIMER_API bool alimerPhysicsShapeIsValid(PhysicsShape shape);
ALIMER_API PhysicsShapeType alimerPhysicsShapeGetType(PhysicsShape shape);
ALIMER_API PhysicsShape alimerPhysicsCreateBoxShape(const float dimensions[3]);

/* Body */
ALIMER_API PhysicsBody alimerPhysicsBodyCreate(PhysicsWorld world, PhysicsShape shape);
ALIMER_API void alimerPhysicsBodyAddRef(PhysicsBody body);
ALIMER_API void alimerPhysicsBodyRelease(PhysicsBody body);
ALIMER_API bool alimerPhysicsBodyIsValid(PhysicsBody body);

#endif /* _ALIMER_PHYSICS_H */
