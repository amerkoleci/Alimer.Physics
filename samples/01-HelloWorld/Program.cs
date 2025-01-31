// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using System.Numerics;
using Alimer.Physics;
using Alimer.SampleFramework;

namespace HelloWorld;

public static class Program
{
    class HelloWorldApp : Application
    {
        const int NumberOfBoxes = 12;

        public HelloWorldApp()
            : base("01 - Hello World")
        {
            _ = CreateFloor(100);

            // add NumberOfBoxes cubes
            for (int i = 0; i < NumberOfBoxes; i++)
            {
                _ = CreateBox(
                    new Vector3(0.5f),
                    new Vector3(0, i * 2 + 0.5f, 0),
                    Quaternion.Identity,
                    PhysicsBodyType.Dynamic
                    );
            } 
        }
    }

    public static unsafe void Main()
    {
        using HelloWorldApp app = new();
        app.Run();
    }
}
