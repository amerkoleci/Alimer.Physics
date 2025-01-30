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
JPH_SUPPRESS_WARNING_POP

// STL includes
#include <cstdarg>

namespace
{
    JPH::Vec3 ToJolt(const float value[3])
    {
        return JPH::Vec3(value[0], value[1], value[2]);
    }

    void FromJolt(const JPH::Vec3& jolt, float result[3])
    {
        result[0] = jolt.GetX();
        result[1] = jolt.GetY();
        result[2] = jolt.GetZ();
    }
}

// Based on: https://github.com/jrouwe/JoltPhysics/blob/master/HelloWorld/HelloWorld.cpp
namespace Layers
{
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}

namespace BroadPhaseLayers
{
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr uint32_t NUM_LAYERS(2);
}

/// Class that determines if two object layers can collide
class JoltObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
    {
        switch (inObject1)
        {
            case Layers::NON_MOVING:
                return inObject2 == Layers::MOVING; // Non moving only collides with moving
            case Layers::MOVING:
                return true; // Moving collides with everything
            default:
                //JPH_ASSERT(false);
                return false;
        }
    }
};

class JoltBroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface
{
private:
    JPH::BroadPhaseLayer objectToBroadPhase[Layers::NUM_LAYERS];

public:
    JoltBroadPhaseLayerInterface()
    {
        // Create a mapping table from object to broad phase layer
        objectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        objectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
    }

    uint32_t GetNumBroadPhaseLayers() const override
    {
        return BroadPhaseLayers::NUM_LAYERS;
    }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
    {
        JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
        return objectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
    {
        switch ((uint8_t)inLayer)
        {
            case (uint8_t)BroadPhaseLayers::NON_MOVING:	return "NON_MOVING";
            case (uint8_t)BroadPhaseLayers::MOVING:		return "MOVING";
            default:
                JPH_ASSERT(false);
                return "INVALID";
        }
    }
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED
};

/// Class that determines if an object layer can collide with a broadphase layer
class JoltObjectVsBroadPhaseLayerFilter final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    JoltObjectVsBroadPhaseLayerFilter() = default;

    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
    {
        switch (inLayer1)
        {
            case Layers::NON_MOVING:
                return inLayer2 == BroadPhaseLayers::MOVING;
            case Layers::MOVING:
                return true;
            default:
                JPH_ASSERT(false);
                return false;
        }

        //return AlimerJoltCollisionFiltering::s_CollisionFilterConfig.IsCollisionEnabled(m_uiCollisionLayer, static_cast<ezUInt32>(inLayer) & 0xFF);
        return true;
    }
};

class JoltBodyActivationListener final : public JPH::BodyActivationListener
{
public:
    void OnBodyActivated([[maybe_unused]] const JPH::BodyID& inBodyID, [[maybe_unused]] JPH::uint64 inBodyUserData) override
    {
        //ALIMER_PROFILE_SCOPE();

        /* Body Activated */
    }

    void OnBodyDeactivated([[maybe_unused]] const JPH::BodyID& inBodyID, [[maybe_unused]] JPH::uint64 inBodyUserData) override
    {
        //ALIMER_PROFILE_SCOPE();

        /* Body Deactivated */
    }
};

class JoltContactListener final : public JPH::ContactListener
{
private:
    static void	GetFrictionAndRestitution(const JPH::Body& inBody, const JPH::SubShapeID& inSubShapeID, float& outFriction, float& outRestitution)
    {
        //ALIMER_PROFILE_SCOPE();

        // Get the material that corresponds to the sub shape ID
        const JPH::PhysicsMaterial* material = inBody.GetShape()->GetMaterial(inSubShapeID);
        if (material == JPH::PhysicsMaterial::sDefault)
        {
            outFriction = inBody.GetFriction();
            outRestitution = inBody.GetRestitution();
        }
        else
        {
            //const auto* phyMaterial = static_cast<const PhysicsMaterial3D*>(material);
            //outFriction = phyMaterial->Friction;
            //outRestitution = phyMaterial->Restitution;
        }
    }

    static void	OverrideContactSettings(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
    {
        //ALIMER_PROFILE_SCOPE();

        // Get the custom friction and restitution for both bodies
        float friction1, friction2, restitution1, restitution2;
        GetFrictionAndRestitution(inBody1, inManifold.mSubShapeID1, friction1, restitution1);
        GetFrictionAndRestitution(inBody2, inManifold.mSubShapeID2, friction2, restitution2);

        // Use the default formulas for combining friction and restitution
        ioSettings.mCombinedFriction = JPH::sqrt(friction1 * friction2);
        ioSettings.mCombinedRestitution = JPH::max(restitution1, restitution2);
    }

public:
    JPH::ValidateResult OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult) override
    {
        //ALIMER_PROFILE_SCOPE();

        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override
    {
        // TODO: Triggers

        OverrideContactSettings(inBody1, inBody2, inManifold, ioSettings);
    }

    void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override
    {
        //ALIMER_PROFILE_SCOPE();

        OverrideContactSettings(inBody1, inBody2, inManifold, ioSettings);
    }

    void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) override
    {
        //ALIMER_PROFILE_SCOPE();

        /* On Collision Exit */
    }
};

struct PhysicsWorldImpl final
{
    std::atomic_uint32_t                refCount;
    JoltObjectLayerPairFilter           objectLayerFilter;
    JoltBroadPhaseLayerInterface	    broadPhaseLayerInterface;
    JoltObjectVsBroadPhaseLayerFilter   objectVsBroadPhaseLayerFilter;
    JPH::PhysicsSystem                  system;
    JoltBodyActivationListener          bodyActivationListener;
    JoltContactListener                 contactListener;
    JPH::ShapeRefC                      emptyShape;
};

struct PhysicsBodyImpl final
{
    std::atomic_uint32_t refCount;
    JPH::Body* handle;
    JPH::BodyID id;
};

struct PhysicsShapeImpl final
{
    std::atomic_uint32_t refCount;
    PhysicsShapeType type;
    JPH::ShapeRefC handle;
    PhysicsBody* body;
};

struct PhysicsMaterialImpl final
{
    std::atomic_uint32_t refCount;
    JPH::PhysicsMaterial* handle;
};

static struct
{
    bool initialized;
    PhysicsConfig config;
    JPH::TempAllocatorImplWithMallocFallback* tempAllocator;
    JPH::JobSystemThreadPool* jobSystem;
} state = {};

namespace
{
    void Log(LogLevel level, const char* message)
    {
        if (!state.config.logCallback)
            return;

        state.config.logCallback(state.config.userdata, level, message);
        if (level == LogLevel_Error) {
            ALIMER_DEBUG_BREAK();
        }
    }

    void LogFormat(LogLevel level, const char* format, ...)
    {
        if (!state.config.logCallback)
            return;

        char message[2048];
        va_list args;
        va_start(args, format);
        vsnprintf(message, sizeof(message), format, args);
        va_end(args);

        state.config.logCallback(state.config.userdata, level, message);
        if (level == LogLevel_Error) {
            ALIMER_DEBUG_BREAK();
        }
    }
}

static void TraceImpl(const char* fmt, ...)
{
    // Format the message
    va_list list;
    va_start(list, fmt);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), fmt, list);
    va_end(list);

    Log(LogLevel_Trace, buffer);
}

#ifdef JPH_ENABLE_ASSERTS
//static JPH_AssertFailureFunc s_AssertFailureFunc = nullptr;

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char* inExpression, const char* inMessage, const char* inFile, uint32_t inLine)
{
    // Print to the TTY
    //alimerLogError(LogCategory_Physics, "%s:%s: (%s) %s", inFile, inLine, inExpression, (inMessage != nullptr ? inMessage : ""));

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

PhysicsWorld alimerPhysicsWorldCreate(const PhysicsWorldConfig* config)
{
    ALIMER_ASSERT(config);

    PhysicsWorldImpl* world = new PhysicsWorldImpl();
    world->refCount.store(1);

    // Init the physics system
    const uint32_t maxBodies = config->maxBodies ? config->maxBodies : 65536;
    const uint32_t maxBodyPairs = config->maxBodyPairs ? config->maxBodyPairs : 65536;
    const uint32_t maxContactConstraints = maxBodies;

    world->system.Init(maxBodies, 0, maxBodyPairs, maxContactConstraints,
        world->broadPhaseLayerInterface,
        world->objectVsBroadPhaseLayerFilter,
        world->objectLayerFilter
    );
    world->system.SetBodyActivationListener(&world->bodyActivationListener);
    world->system.SetContactListener(&world->contactListener);

    JPH::EmptyShapeSettings settings(JPH::Vec3::sZero());
    settings.SetEmbedded();
    JPH::ShapeSettings::ShapeResult shapeResult = settings.Create();
    JPH_ASSERT(shapeResult.IsValid());
    world->emptyShape = shapeResult.Get();
    return world;
}

void alimerPhysicsWorldDestroy(PhysicsWorld world)
{
    if (world->refCount.fetch_sub(1, std::memory_order_release) == 1)
    {
        delete world;
    }
}

uint32_t alimerPhysicsWorldGetBodyCount(PhysicsWorld world)
{
    return world->system.GetNumBodies();
}

uint32_t alimerPhysicsWorldGetActiveBodyCount(PhysicsWorld world)
{
    return world->system.GetNumActiveBodies(JPH::EBodyType::RigidBody);
}

void alimerPhysicsWorldGetGravity(PhysicsWorld world, float gravity[3])
{
    FromJolt(world->system.GetGravity(), gravity);
}

void alimerPhysicsWorldSetGravity(PhysicsWorld world, const float gravity[3])
{
    world->system.SetGravity(ToJolt(gravity));
}

bool alimerPhysicsWorldUpdate(PhysicsWorld world, float deltaTime, int collisionSteps)
{
    JPH::EPhysicsUpdateError error = world->system.Update(
        deltaTime,
        collisionSteps,
        state.tempAllocator,
        state.jobSystem
    );
    return error == JPH::EPhysicsUpdateError::None;
}

/* Shape */
void alimerPhysicsShapeAddRef(PhysicsShape shape)
{
    shape->refCount.fetch_add(1, std::memory_order_relaxed);
}

void alimerPhysicsShapeRelease(PhysicsShape shape)
{
    if (shape->refCount.fetch_sub(1, std::memory_order_release) == 1)
    {
        delete shape;
    }
}

bool alimerPhysicsShapeIsValid(PhysicsShape shape)
{
    return shape && shape->handle != nullptr;
}

PhysicsShapeType alimerPhysicsShapeGetType(PhysicsShape shape)
{
    return shape->type;
}

PhysicsShape alimerPhysicsCreateBoxShape(const float dimensions[3])
{
    //ALIMER_Check(dimensions[0] > 0.f && dimensions[1] > 0.f && dimensions[2] > 0.f, "BoxShape dimensions must be positive");

    PhysicsShapeImpl* shape = new PhysicsShapeImpl();
    shape->refCount.store(1);

    shape->type = PhysicsShapeType_Box;
    const JPH::Vec3 halfExtent = { dimensions[0] / 2.f, dimensions[1] / 2.f, dimensions[2] / 2.f };
    float shortestSide = std::min(dimensions[0], std::min(dimensions[1], dimensions[2]));
    float convexRadius = std::min(shortestSide * .1f, .05f);

    JPH::BoxShapeSettings settings(halfExtent, convexRadius);
    settings.SetEmbedded();
    JPH::ShapeSettings::ShapeResult shapeResult = settings.Create();
    if (!shapeResult.IsValid())
    {
        LogFormat(LogLevel_Error, "CreateBox failed with %s", shapeResult.GetError().c_str());
        return nullptr;
    }

    shape->handle = shapeResult.Get();
    //shape->handle->SetUserData((uint64_t)(uintptr_t)shape);
    return shape;
}

/* Body */
PhysicsBody alimerPhysicsBodyCreate(PhysicsWorld world, PhysicsShape shape)
{
    const uint32_t count = world->system.GetNumBodies();
    const uint32_t limit = world->system.GetMaxBodies();
    if (count >= limit)
    {
        LogFormat(LogLevel_Error, "Too many bodies, limit %s reached!", limit);
        return nullptr;
    }

    ALIMER_ASSERT(shape);
    //if (!shape->body)
    //{
    //    alimerLogError(LogCategory_Physics, "Shape is already attached to a collider!");
    //    return nullptr;
    //}

    JPH::BodyInterface& bodyInterface = world->system.GetBodyInterface();

    JPH::RVec3 position = JPH::RVec3::sZero();
    JPH::Quat rotation = JPH::Quat::sIdentity();
    JPH::EMotionType type = shape && shape->handle->MustBeStatic() ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic;
    JPH::ObjectLayer objectLayer = type == JPH::EMotionType::Static ? Layers::NON_MOVING : Layers::MOVING;
    JPH::BodyCreationSettings settings(
        shape ? shape->handle : world->emptyShape,
        position,
        rotation,
        type,
        objectLayer
    );

    PhysicsBodyImpl* body = new PhysicsBodyImpl();
    body->refCount.store(1);
    body->handle = bodyInterface.CreateBody(settings);
    body->id = body->handle->GetID();
    body->handle->SetUserData((uint64_t)(uintptr_t)body);

    // Add it to the world
    bodyInterface.AddBody(body->id, JPH::EActivation::Activate);
    return body;
}

void alimerPhysicsBodyAddRef(PhysicsBody body)
{
    body->refCount.fetch_add(1, std::memory_order_relaxed);
}

void alimerPhysicsBodyRelease(PhysicsBody body)
{
    if (body->refCount.fetch_sub(1, std::memory_order_release) == 1)
    {
        delete body;
    }
}

bool alimerPhysicsBodyIsValid(PhysicsBody body)
{
    return body && body->handle != nullptr;
}
