// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

namespace Alimer.Physics;

public readonly struct PhysicsWorldConfig
{
    public readonly int MaxBodies;
    public readonly int MaxBodyPairs;

    public PhysicsWorldConfig(int maxBodies = 0, int maxBodyPairs = 0)
    {
        MaxBodies = maxBodies;
        MaxBodyPairs = maxBodyPairs;
    }
}
