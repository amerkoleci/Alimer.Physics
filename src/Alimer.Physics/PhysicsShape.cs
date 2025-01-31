// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using System.Numerics;
using static Alimer.Physics.PhysicsApi;

namespace Alimer.Physics;

public sealed class PhysicsShape : NativeObject
{
    private PhysicsShape(nint handle, bool ownsHandle = true)
        : base(handle, ownsHandle)
    {

    }

    public bool IsValid => alimerPhysicsShapeIsValid(Handle);
    public PhysicsShapeType ShapeType => alimerPhysicsShapeGetType(Handle);

    protected override void DisposeNative()
    {
        alimerPhysicsShapeRelease(Handle);
    }

    public static PhysicsShape CreateBox(in Vector3 halfExtents)
    {
        return new(alimerPhysicsShapeCreateBox(in halfExtents));
    }

    public static PhysicsShape CreateSphere(float radius)
    {
        return new(alimerPhysicsShapeCreateSphere(radius));
    }
}
