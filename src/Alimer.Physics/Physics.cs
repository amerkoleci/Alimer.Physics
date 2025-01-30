// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using System.Runtime.InteropServices;
using static Alimer.Physics.PhysicsApi;

namespace Alimer.Physics;

public static class Physics
{
    private static bool s_initialized;

    public static unsafe bool Initialize()
    {
        if (s_initialized)
            return true;

        PhysicsConfig config = new()
        {
            logCallback = &OnNativeMessageCallback
        };

        if (!alimerPhysicsInit(in config))
        {
            return false;
        }

        s_initialized = true;
        return true;
    }

    public static void Shutdown()
    {
        if (!s_initialized)
            return;

        alimerPhysicsShutdown();
        s_initialized = false;
    }

    [UnmanagedCallersOnly]
    private static unsafe void OnNativeMessageCallback(nint userdata, LogLevel level, byte* messagePtr)
    {
        string? message = ConvertToManaged(messagePtr);
    }
}
