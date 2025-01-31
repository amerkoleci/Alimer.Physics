// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using NativePhysicsWorld = System.IntPtr;
using NativePhysicsShape = System.IntPtr;
using NativePhysicsBody = System.IntPtr;
using System.Numerics;

namespace Alimer.Physics;

internal static unsafe partial class PhysicsApi
{
    public const string LibName = "alimer_physics";

    public enum LogLevel
    {
        Error = 0,
        Warn = 1,
        Info = 2,
        Trace = 3,
    }

    public struct PhysicsConfig
    {
        public nint userdata;
        public delegate* unmanaged<nint, LogLevel, byte*, void> logCallback;

        public uint tempAllocatorInitSize;
        public uint maxPhysicsJobs;
        public uint maxPhysicsBarriers;
    }

    public struct PhysicsBodyDesc
    {
        public PhysicsBodyType type;
        public PhysicsBodyTransform initialTransform;
        public float mass;
        public float friction;
        public float restitution;
        public float linearDamping;
        public float angularDamping;
        public float gravityScale;
        public bool isSensor;
        public bool allowSleeping;
        public bool continuous;
        public nuint shapeCount;
        public NativePhysicsShape* shapes;
    }

    [LibraryImport(LibName)]
    [return: MarshalAs(UnmanagedType.U1)]
    public static partial bool alimerPhysicsInit(in PhysicsConfig config);

    [LibraryImport(LibName)]
    public static partial void alimerPhysicsShutdown();

    /* World */
    [LibraryImport(LibName)]
    public static partial NativePhysicsWorld alimerPhysicsWorldCreate(in PhysicsWorldConfig config);
    [LibraryImport(LibName)]
    public static partial void alimerPhysicsWorldDestroy(NativePhysicsWorld world);
    [LibraryImport(LibName)]
    public static partial int alimerPhysicsWorldGetBodyCount(NativePhysicsWorld world);
    [LibraryImport(LibName)]
    public static partial int alimerPhysicsWorldGetActiveBodyCount(NativePhysicsWorld world);
    [LibraryImport(LibName)]
    public static partial void alimerPhysicsWorldGetGravity(NativePhysicsWorld world, out Vector3 gravity);
    [LibraryImport(LibName)]
    public static partial void alimerPhysicsWorldSetGravity(NativePhysicsWorld world, in Vector3 value);

    [LibraryImport(LibName)]
    [return: MarshalAs(UnmanagedType.U1)]
    public static partial bool alimerPhysicsWorldUpdate(NativePhysicsWorld world, float deltaTime, int collisionSteps);


    [LibraryImport(LibName)]
    public static partial void alimerPhysicsWorldOptimizeBroadPhase(NativePhysicsWorld world);

    /* Shape */
    [LibraryImport(LibName)]
    public static partial void alimerPhysicsShapeAddRef(NativePhysicsShape shape);
    [LibraryImport(LibName)]
    public static partial void alimerPhysicsShapeRelease(NativePhysicsShape shape);

    [LibraryImport(LibName)]
    [return: MarshalAs(UnmanagedType.U1)]
    public static partial bool alimerPhysicsShapeIsValid(NativePhysicsShape shape);

    [LibraryImport(LibName)]
    public static partial PhysicsShapeType alimerPhysicsShapeGetType(NativePhysicsShape shape);

    [LibraryImport(LibName)]
    public static partial NativePhysicsShape alimerPhysicsShapeCreateBox(in Vector3 dimensions);

    [LibraryImport(LibName)]
    public static partial NativePhysicsShape alimerPhysicsShapeCreateSphere(float radius);

    /* Body */
    [LibraryImport(LibName)]
    public static partial void alimerPhysicsBodyDescInit(PhysicsBodyDesc* desc);


    [LibraryImport(LibName)]
    public static partial NativePhysicsBody alimerPhysicsBodyCreate(NativePhysicsWorld world, PhysicsBodyDesc* desc);

    [LibraryImport(LibName)]
    public static partial void alimerPhysicsBodyAddRef(NativePhysicsBody body);

    [LibraryImport(LibName)]
    public static partial void alimerPhysicsBodyRelease(NativePhysicsBody body);

    [LibraryImport(LibName)]
    [return: MarshalAs(UnmanagedType.U1)]
    public static partial bool alimerPhysicsBodyIsValid(NativePhysicsBody body);

    [LibraryImport(LibName)]
    public static partial void alimerPhysicsBodyGetWorldTransform(NativePhysicsBody body, PhysicsBodyTransform* result);

    #region Marshal
    /// <summary>Converts an unmanaged string to a managed version.</summary>
    /// <param name="unmanaged">The unmanaged string to convert.</param>
    /// <returns>A managed string.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static string? ConvertToManaged(byte* unmanaged) => Utf8CustomMarshaller.ConvertToManaged(unmanaged);

    /// <summary>Converts an unmanaged string to a managed version.</summary>
    /// <param name="unmanaged">The unmanaged string to convert.</param>
    /// <returns>A managed string.</returns>
    public static string? ConvertToManaged(byte* unmanaged, int maxLength) => Utf8CustomMarshaller.ConvertToManaged(unmanaged, maxLength);
    #endregion
}
