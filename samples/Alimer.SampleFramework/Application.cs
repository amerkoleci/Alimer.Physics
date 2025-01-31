// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

using System.Diagnostics;
using System.Numerics;
using Alimer.Physics;
using Raylib_cs;
using static Raylib_cs.Raylib;

namespace Alimer.SampleFramework;

public abstract class Application : DisposableObject
{
    private const int MaxBodies = 65536;
    private const int MaxBodyPairs = 65536;

    protected const int TargetFPS = 60;
    protected PhysicsWorldConfig _config;
    protected readonly HashSet<PhysicsBody> _ignoreDrawBodies = [];

    protected Application(string title, int width = 1200, int height = 800)
    {
        if (!Physics.Physics.Initialize())
        {
            return;
        }

        _config = new PhysicsWorldConfig(MaxBodies, MaxBodyPairs);

        //SetupCollisionFiltering();
        PhysicsWorld = new(in _config);

#if TODO
        // ContactListener
        PhysicsSystem.OnContactValidate += OnContactValidate;
        PhysicsSystem.OnContactAdded += OnContactAdded;
        PhysicsSystem.OnContactPersisted += OnContactPersisted;
        PhysicsSystem.OnContactRemoved += OnContactRemoved;
        // BodyActivationListener
        PhysicsSystem.OnBodyActivated += OnBodyActivated;
        PhysicsSystem.OnBodyDeactivated += OnBodyDeactivated; 
#endif

        // set a hint for anti-aliasing
        SetConfigFlags(ConfigFlags.Msaa4xHint);

        InitWindow(width, height, title);

        // 60 fps target
        SetTargetFPS(TargetFPS);

        // Create the main camera
        MainCamera = new()
        {
            Position = new Vector3(-20.0f, 8.0f, 10.0f),
            Target = new Vector3(0.0f, 4.0f, 0.0f),
            Up = new Vector3(0.0f, 1.0f, 0.0f),
            FovY = 45.0f,
            Projection = CameraProjection.Perspective
        };

        // dynamically create a plane model
        Texture2D texture = GenCheckedTexture(10, 1, Color.LightGray, Color.Gray);
        Model planeModel = LoadModelFromMesh(GenMeshPlane(24, 24, 1, 1));
        SetMaterialTexture(ref planeModel, 0, MaterialMapIndex.Diffuse, ref texture);
        PlaneModel = planeModel;

        // dynamically create a box model
        var boxTexture = GenCheckedTexture(2, 1, Color.White, Color.Magenta);
        BoxMesh = GenMeshCube(1, 1, 1);
        Material boxMat = LoadMaterialDefault();
        SetMaterialTexture(ref boxMat, MaterialMapIndex.Diffuse, boxTexture);
        BoxMaterial = boxMat;
    }

    public PhysicsWorld PhysicsWorld { get; private set; }

    public Camera3D MainCamera { get; }
    public Model PlaneModel { get; }
    public Mesh BoxMesh { get; }

    public Material BoxMaterial { get; }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            //foreach (BodyID bodyID in _bodies)
            //{
            //    BodyInterface.RemoveAndDestroyBody(bodyID);
            //}
            //_bodies.Clear();

            PhysicsWorld.Dispose();
            Physics.Physics.Shutdown();
        }
    }

    public void Run()
    {
        // Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
        // You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
        // Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
        PhysicsWorld.OptimizeBroadPhase();

        // If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
        const int collisionSteps = 1;

        float deltaTime = 1.0f / TargetFPS;

        while (!WindowShouldClose())
        {
            // Step the world
            bool success = PhysicsWorld.Update(deltaTime, collisionSteps);
            Debug.Assert(success);

            BeginDrawing();
            ClearBackground(Color.Blue);

            BeginMode3D(MainCamera);
            DrawModel(PlaneModel, Vector3.Zero, 1.0f, Color.White);

            foreach (PhysicsBody body in PhysicsWorld.Bodies)
            {
                if (_ignoreDrawBodies.Contains(body))
                    continue;

                var transform = body.WorldTransform;

                //Vector3 pos = BodyInterface.GetPosition(bodyID);
                //Quaternion rot = BodyInterface.GetRotation(bodyID);
                Matrix4x4 ori = Matrix4x4.CreateFromQuaternion(transform.Rotation);
                Matrix4x4 worldTransform = new(
                    ori.M11, ori.M12, ori.M13, transform.Position.X,
                    ori.M21, ori.M22, ori.M23, transform.Position.Y,
                    ori.M31, ori.M32, ori.M33, transform.Position.Z,
                    0, 0, 0, 1.0f);

                // Raylib uses column major matrix
                //Matrix4x4 worldTransform = body.WorldTransform;
                //Matrix4x4 drawTransform = Matrix4x4.Transpose(worldTransform);
                DrawMesh(BoxMesh, BoxMaterial, worldTransform);
            }

            EndMode3D();

            DrawText($"{GetFPS()} fps", 10, 10, 20, Color.White);

            EndDrawing();
        }

        CloseWindow();
    }

    protected PhysicsBody CreateFloor(float size)
    {
        PhysicsShape shape = PhysicsShape.CreateBox(new Vector3(size, 5.0f, size));
        PhysicsBodyDescription bodyDescription = new()
        {
            Type = PhysicsBodyType.Static,
            InitialTransform = new PhysicsBodyTransform(new Vector3(0, -5.0f, 0.0f), Quaternion.Identity)
        };

        PhysicsBody body = new(PhysicsWorld, in bodyDescription, shape);
        _ignoreDrawBodies.Add(body);
        return body;
    }

    protected PhysicsBody CreateBox(in Vector3 extents,
        in Vector3 position,
        in Quaternion rotation,
        PhysicsBodyType type)
    {
        PhysicsShape shape = PhysicsShape.CreateBox(in extents);
        PhysicsBodyDescription bodyDescription = new()
        {
            Type = type,
            InitialTransform = new PhysicsBodyTransform(position, rotation)
        };

        PhysicsBody body = new(PhysicsWorld, in bodyDescription, shape);
        return body;
    }

    protected PhysicsBody CreateSphere(float radius,
        in Vector3 position,
        in Quaternion rotation,
        PhysicsBodyType type)
    {
        PhysicsShape shape = PhysicsShape.CreateSphere(radius);
        PhysicsBodyDescription bodyDescription = new()
        {
            Type = type,
            InitialTransform = new PhysicsBodyTransform(position, rotation)
        };
        PhysicsBody body = new(PhysicsWorld, in bodyDescription, shape);
        return body;
    }

    #region Raylib
    protected static Texture2D GenCheckedTexture(int size, int checks, Color colorA, Color colorB)
    {
        Image imageMag = GenImageChecked(size, size, checks, checks, colorA, colorB);
        Texture2D textureMag = LoadTextureFromImage(imageMag);
        UnloadImage(imageMag);
        return textureMag;
    }
    #endregion
}
