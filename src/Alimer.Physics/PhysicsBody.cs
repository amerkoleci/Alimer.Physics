// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using static Alimer.Physics.PhysicsApi;

namespace Alimer.Physics;

public sealed class PhysicsBody : NativeObject
{
    private readonly List<PhysicsShape> _shapes = new();

    public unsafe PhysicsBody(PhysicsWorld world, in PhysicsBodyDescription description, params PhysicsShape[] shapes)
    {
        PhysicsBodyDesc nativeDesc = new();
        nativeDesc.type = description.Type;
        nativeDesc.initialTransform = description.InitialTransform;
        nativeDesc.mass = description.Mass;
        nativeDesc.friction = description.Friction;
        nativeDesc.restitution = description.Restitution;
        nativeDesc.linearDamping = description.LinearDamping;
        nativeDesc.angularDamping = description.AngularDamping;
        nativeDesc.gravityScale = description.GravityScale;
        nativeDesc.isSensor = description.IsSensor;
        nativeDesc.allowSleeping = description.AllowSleeping;
        nativeDesc.continuous = description.Continuous;
        nativeDesc.shapeCount = (nuint)shapes.Length;

        IntPtr* shapesPtr = stackalloc IntPtr[shapes.Length];
        for (int i = 0; i < shapes.Length; i++)
        {
            shapesPtr[i] = shapes[i].Handle;
        }
        nativeDesc.shapes = shapesPtr;
        Handle = alimerPhysicsBodyCreate(world.Handle, &nativeDesc);

        World = world;
        world.AddBody(this);
        _shapes.AddRange(shapes);
    }

    public PhysicsWorld World { get; }
    public bool IsValid => alimerPhysicsBodyIsValid(Handle);
    public unsafe PhysicsBodyTransform WorldTransform
    {
        get
        {
            PhysicsBodyTransform transform;
            alimerPhysicsBodyGetWorldTransform(Handle, &transform);
            return transform;
        }
    }

    protected override void DisposeNative()
    {
        World.RemoveBody(this);
        alimerPhysicsBodyRelease(Handle);
    }
}
