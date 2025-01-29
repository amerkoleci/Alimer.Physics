// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

#include "alimer_internal.h"
#include <Jolt/Core/Core.h>

JPH_SUPPRESS_WARNING_PUSH
JPH_SUPPRESS_WARNINGS
#include "Jolt/Jolt.h"
#include "Jolt/RegisterTypes.h"
#include "Jolt/Core/Factory.h"
#include "Jolt/Core/TempAllocator.h"
#include "Jolt/Core/JobSystemThreadPool.h"
#include "Jolt/Physics/PhysicsSettings.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Body/BodyActivationListener.h"
#include "Jolt/Physics/Collision/Shape/EmptyShape.h"
#include "Jolt/Physics/Collision/Shape/BoxShape.h"
#include "Jolt/Physics/Collision/Shape/SphereShape.h"
#include "Jolt/Physics/Collision/Shape/CapsuleShape.h"
#include "Jolt/Physics/Collision/Shape/CylinderShape.h"

// STL includes
#include <cstdarg>

static struct
{
    bool initialized;
    PhysicsConfig config;
    JPH::TempAllocatorImplWithMallocFallback* tempAllocator;
    JPH::JobSystemThreadPool* jobSystem;
} state = {};

#define LOG(s) if (state.config.logCallback) state.config.fnLog(state.config.userdata, s)

JPH_SUPPRESS_WARNING_POP

static void TraceImpl(const char* fmt, ...)
{
    // Format the message
    va_list list;
    va_start(list, fmt);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), fmt, list);
    va_end(list);

    //alimerLogTrace(LogCategory_Physics, "%s", buffer);
}

#ifdef JPH_ENABLE_ASSERTS
//static JPH_AssertFailureFunc s_AssertFailureFunc = nullptr;

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char* inExpression, const char* inMessage, const char* inFile, uint32_t inLine)
{
    // Print to the TTY
    alimerLogError(LogCategory_Physics, "%s:%s: (%s) %s", inFile, inLine, inExpression, (inMessage != nullptr ? inMessage : ""));

    // Breakpoint
    return true;
};

#endif // JPH_ENABLE_ASSERTS

bool alimerPhysicsInit(const PhysicsConfig* config)
{
    ALIMER_ASSERT(config);

    if (state.initialized)
        return true;

    state.config = *config;

    JPH::RegisterDefaultAllocator();

    // TODO
    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

    // Create a factory
    JPH::Factory::sInstance = new JPH::Factory();

    // Register all Jolt physics types
    JPH::RegisterTypes();

    const uint32_t tempAllocatorSize = config->tempAllocatorInitSize > 0 ? config->tempAllocatorInitSize : 8 * 1024 * 1024;
    const uint32_t maxPhysicsJobs = config->maxPhysicsJobs > 0 ? config->maxPhysicsJobs : JPH::cMaxPhysicsJobs;
    const uint32_t maxPhysicsBarriers = config->maxPhysicsBarriers > 0 ? config->maxPhysicsBarriers : JPH::cMaxPhysicsBarriers;

    // Init temp allocator
    state.tempAllocator = new JPH::TempAllocatorImplWithMallocFallback(tempAllocatorSize);

    // Init Job system.
    state.jobSystem = new JPH::JobSystemThreadPool(maxPhysicsJobs, maxPhysicsBarriers);

    state.initialized = true;
    return true;
}

void alimerPhysicsShutdown(void)
{
    if (!state.initialized)
        return;

    delete state.jobSystem; state.jobSystem = nullptr;
    delete state.tempAllocator; state.tempAllocator = nullptr;

    // Unregisters all types with the factory and cleans up the default material
    JPH::UnregisterTypes();

    // Destroy the factory
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;

    state.initialized = false;
    memset(&state, 0, sizeof(state));
}
