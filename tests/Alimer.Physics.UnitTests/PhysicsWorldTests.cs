// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using NUnit.Framework;

namespace Alimer.Physics.UnitTests;

[TestFixture(TestOf = typeof(PhysicsWorld))]
public class PhysicsWorldTests
{
    [SetUp]
    public void SetUp()
    {
        _ = Physics.Initialize();
    }

    [TearDown]
    public void TearDown()
    {
        Physics.Shutdown();
    }

    [Test]
    public static void DefaultCreationTest()
    {
        PhysicsWorld world = new();

        Assert.That(() => world.Handle, Is.Not.EqualTo(IntPtr.Zero));
        Assert.That(() => world.IsDisposed, Is.EqualTo(false));

        world.Dispose();
        Assert.That(() => world.Handle, Is.EqualTo(IntPtr.Zero));
        Assert.That(() => world.IsDisposed, Is.EqualTo(true));
    }
}
