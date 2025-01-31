// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using System.Numerics;

namespace Alimer.Physics;

public record struct PhysicsBodyTransform
{
    public Vector3 Position;
    public Quaternion Rotation;

    public static PhysicsBodyTransform Identity => new(Vector3.Zero, Quaternion.Identity);

    public PhysicsBodyTransform(Vector3 position, Quaternion rotation)
    {
        Position = position;
        Rotation = rotation;
    }
}
