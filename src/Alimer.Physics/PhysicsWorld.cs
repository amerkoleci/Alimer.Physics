// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using static Alimer.Physics.PhysicsApi;

namespace Alimer.Physics;

public sealed class PhysicsWorld : NativeObject
{
    public PhysicsWorld()
        : this(new PhysicsWorldConfig())
    {

    }

    public PhysicsWorld(in PhysicsWorldConfig config)
        : base(alimerPhysicsWorldCreate(in config))
    {

    }

    protected override void DisposeNative()
    {
        alimerPhysicsWorldDestroy(Handle);
    }
}
