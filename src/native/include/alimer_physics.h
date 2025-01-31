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

#ifdef __cplusplus
#   define DEFAULT_INITIALIZER(x) = x
#else
#   define DEFAULT_INITIALIZER(x)
#endif

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
    LogLevel_Trace = 3,

    LogLevel_Count,
    _LogLevel_Force32 = 0x7FFFFFFF
} LogLevel;

typedef enum PhysicsBodyType {
    PhysicsBodyType_Static,
    PhysicsBodyType_Kinematic,
    PhysicsBodyType_Dynamic,

    _PhysicsBodyType_Count,
    _PhysicsBodyType_Force32 = 0x7FFFFFFF
} PhysicsBodyType;

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

typedef struct Vec3 {
    float x;
    float y;
    float z;
} Vec3;

typedef struct Quat {
    float x;
    float y;
    float z;
    float w;
} Quat;

typedef struct PhysicsBodyTransform {
    Vec3 position;
    Quat rotation;
} PhysicsBodyTransform;

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

typedef struct PhysicsBodyDesc {
    PhysicsBodyType type;
    PhysicsBodyTransform initialTransform;
    float mass DEFAULT_INITIALIZER(1.0f);
    float friction DEFAULT_INITIALIZER(0.2f);
    float restitution DEFAULT_INITIALIZER(0.0f);
    float linearDamping DEFAULT_INITIALIZER(0.05f);
    float angularDamping DEFAULT_INITIALIZER(0.05f);
    float gravityScale DEFAULT_INITIALIZER(1.0f);
    bool isSensor DEFAULT_INITIALIZER(false);
    bool allowSleeping DEFAULT_INITIALIZER(true);
    bool continuous DEFAULT_INITIALIZER(false);
    size_t shapeCount DEFAULT_INITIALIZER(0);
    PhysicsShape const* shapes DEFAULT_INITIALIZER(nullptr);
} PhysicsBodyDesc;

ALIMER_API bool alimerPhysicsInit(const PhysicsConfig* config);
ALIMER_API void alimerPhysicsShutdown(void);

/* World */
ALIMER_API PhysicsWorld alimerPhysicsWorldCreate(const PhysicsWorldConfig* config);
ALIMER_API void alimerPhysicsWorldDestroy(PhysicsWorld world);
ALIMER_API uint32_t alimerPhysicsWorldGetBodyCount(PhysicsWorld world);
ALIMER_API uint32_t alimerPhysicsWorldGetActiveBodyCount(PhysicsWorld world);
ALIMER_API void alimerPhysicsWorldGetGravity(PhysicsWorld world, Vec3* result);
ALIMER_API void alimerPhysicsWorldSetGravity(PhysicsWorld world, const Vec3* gravity);
ALIMER_API bool alimerPhysicsWorldUpdate(PhysicsWorld world, float deltaTime, int collisionSteps);
ALIMER_API void alimerPhysicsWorldOptimizeBroadPhase(PhysicsWorld world);

/* Shape */
ALIMER_API void alimerPhysicsShapeAddRef(PhysicsShape shape);
ALIMER_API void alimerPhysicsShapeRelease(PhysicsShape shape);
ALIMER_API bool alimerPhysicsShapeIsValid(PhysicsShape shape);
ALIMER_API PhysicsShapeType alimerPhysicsShapeGetType(PhysicsShape shape);
ALIMER_API PhysicsShape alimerPhysicsShapeCreateBox(const Vec3* extents);
ALIMER_API PhysicsShape alimerPhysicsShapeCreateSphere(float radius);

/* Body */
ALIMER_API void alimerPhysicsBodyDescInit(PhysicsBodyDesc* desc);
ALIMER_API PhysicsBody alimerPhysicsBodyCreate(PhysicsWorld world, const PhysicsBodyDesc* desc);
ALIMER_API void alimerPhysicsBodyAddRef(PhysicsBody body);
ALIMER_API void alimerPhysicsBodyRelease(PhysicsBody body);
ALIMER_API bool alimerPhysicsBodyIsValid(PhysicsBody body);
ALIMER_API void alimerPhysicsBodyGetWorldTransform(PhysicsBody body, PhysicsBodyTransform* result);

#endif /* _ALIMER_PHYSICS_H */
