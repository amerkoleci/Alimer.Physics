// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

namespace Alimer.Physics;

public record struct PhysicsBodyDescription
{
    public PhysicsBodyDescription()
    {
    }

    public PhysicsBodyType Type = PhysicsBodyType.Dynamic;
    public PhysicsBodyTransform InitialTransform = PhysicsBodyTransform.Identity;
    public float Mass = 1.0f;
    public float Friction = 0.2f;
    public float Restitution = 0.0f;
    public float LinearDamping = 0.05f;
    public float AngularDamping = 0.05f;
    public float GravityScale = 1.0f;
    public bool IsSensor = false;
    public bool AllowSleeping = true;
    public bool Continuous = false;
}
