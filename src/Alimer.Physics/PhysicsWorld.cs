// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using System.Numerics;
using static Alimer.Physics.PhysicsApi;

namespace Alimer.Physics;

public sealed class PhysicsWorld : NativeObject
{
    private readonly List<PhysicsBody> _bodies = [];

    public PhysicsWorld()
        : this(new PhysicsWorldConfig())
    {

    }

    public PhysicsWorld(in PhysicsWorldConfig config)
        : base(alimerPhysicsWorldCreate(in config))
    {

    }

    public IEnumerable<PhysicsBody> Bodies => _bodies;
    public int ActiveBodyCount => alimerPhysicsWorldGetActiveBodyCount(Handle);
    public Vector3 Gravity
    {
        get
        {
            alimerPhysicsWorldGetGravity(Handle, out Vector3 value);
            return value;
        }
        set => alimerPhysicsWorldSetGravity(Handle, in value);
    }

    protected override void DisposeNative()
    {
        foreach(PhysicsBody body in _bodies)
        {
            body.Dispose();
        }
        _bodies.Clear();
        alimerPhysicsWorldDestroy(Handle);
    }

    public bool Update(float deltaTime, int collisionSteps)
    {
        return alimerPhysicsWorldUpdate(Handle, deltaTime, collisionSteps);
    }

    public void OptimizeBroadPhase() => alimerPhysicsWorldOptimizeBroadPhase(Handle);

    internal void AddBody(PhysicsBody body) => _bodies.Add(body);
    internal bool RemoveBody(PhysicsBody body) => _bodies.Remove(body);
}
