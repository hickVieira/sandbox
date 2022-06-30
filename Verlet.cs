using System.Collections.Generic;
using System.Numerics;
using Raylib_cs;

/*
TODO:
    - springy softbody-like joints
    - quadtree
    - box shape
    - capsule shape
*/

namespace Sandbox
{
    public static partial class Programs
    {
        // following pezza's work tutorial
        public class Verlet
        {
            public class VerletVertex
            {
                public Vector2 _position;
                public float _mass;
                public float _radius;
                private Vector2 _position_old;
                private Vector2 _acceleration;

                public VerletVertex(Vector2 position, float mass, float radius)
                {
                    this._position = position;
                    this._mass = mass;
                    this._radius = radius;
                    this._position_old = position;
                    this._acceleration = Vector2.Zero;
                }

                public void Render()
                {
                    Raylib.DrawCircle((int)_position.X, (int)_position.Y, _radius, Color.BLACK);
                }

                public void Update(float deltaTime)
                {
                    Vector2 velocity = _position - _position_old;
                    _position_old = _position;
                    _position = _position + velocity + _acceleration * deltaTime * deltaTime;
                    _acceleration = Vector2.Zero;
                }

                public void AddAcceleration(Vector2 accel)
                {
                    _acceleration += accel;
                }

                public void AddForce(Vector2 force)
                {
                    _position_old = _position_old - force;
                }

                public void Collide(VerletVertex otherVerletVertex)
                {
                    Vector2 collisionVector = _position - otherVerletVertex._position;
                    float collisionLength = collisionVector.Length();
                    float collisionDistance = collisionLength - _radius - otherVerletVertex._radius;
                    if (collisionDistance < 0)
                    {
                        float penetration = System.Math.Abs(collisionDistance);
                        Vector2 normal = collisionVector / collisionLength;
                        _position += (1 / (1 + _mass)) * normal * penetration;
                        otherVerletVertex._position -= (1 / (1 + otherVerletVertex._mass)) * normal * penetration;
                    }
                }
            }

            public class VerletEdge
            {
                public VerletVertex _verletVertex0;
                public VerletVertex _verletVertex1;
                public float _distance;

                public VerletEdge(VerletVertex verletVertex0, VerletVertex verletVertex1)
                {
                    this._verletVertex0 = verletVertex0;
                    this._verletVertex1 = verletVertex1;
                    this._distance = (verletVertex0._radius + verletVertex1._radius) + float.Epsilon;
                }

                public void Render()
                {
                    Raylib.DrawLine((int)_verletVertex0._position.X, (int)_verletVertex0._position.Y, (int)_verletVertex1._position.X, (int)_verletVertex1._position.Y, Color.GRAY);
                }

                public void Apply()
                {
                    Vector2 linkVector = _verletVertex0._position - _verletVertex1._position;
                    float linkLength = linkVector.Length();
                    float linkDistance = _distance - linkLength;
                    Vector2 normal = linkVector / linkLength;
                    _verletVertex0._position += 0.5f * normal * linkDistance;
                    _verletVertex1._position -= 0.5f * normal * linkDistance;
                }
            }

            public class VerletSolver
            {
                public List<VerletVertex> _verletVertices;
                public List<VerletEdge> _verletEdges;

                readonly Vector2 _gravity = new Vector2(0, 100);
                const uint _subStepsCount = 4;
                const float _boundaryRadius = 50;
                const float _gravityForce = 9.81f;
                const float _gravityDensity = 5520;

                public VerletSolver()
                {
                    _verletVertices = new List<VerletVertex>();
                    _verletEdges = new List<VerletEdge>();
                }

                public void Render()
                {
                    Raylib.DrawCircle(0, 0, _boundaryRadius, Color.WHITE);
                    for (int i = 0; i < _verletVertices.Count; i++)
                        _verletVertices[i].Render();
                    for (int i = 0; i < _verletEdges.Count; i++)
                        _verletEdges[i].Render();
                }

                public void Solve(float deltaTime)
                {
                    float subDeltaTime = deltaTime / _subStepsCount;
                    for (int i = 0; i < _subStepsCount; i++)
                    {
                        ApplyCenterGravity();
                        SolveBoundaries();
                        SolveJoints();
                        SolveCollisions();
                        UpdateVerlet(subDeltaTime);
                    }
                }

                void ApplyAcceleration(Vector2 accel)
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                        _verletVertices[i].AddAcceleration(accel);
                }

                void ApplyCenterGravity()
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                    {
                        VerletVertex verletVertex = _verletVertices[i];
                        Vector2 toCenterVector = Vector2.Zero - verletVertex._position;
                        float toCenterLength = toCenterVector.Length();
                        Vector2 toCenterDireciton = toCenterVector / toCenterLength;
                        Vector2 force = (verletVertex._mass * toCenterDireciton * _gravityForce * _boundaryRadius * _gravityDensity) / (toCenterLength * toCenterLength);
                        verletVertex.AddAcceleration(force);
                    }
                }

                void SolveBoundaries()
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                    {
                        float positionLength = _verletVertices[i]._position.Length();
                        Vector2 positionDirection = _verletVertices[i]._position / positionLength;
                        float difference = (positionLength - _verletVertices[i]._radius) - _boundaryRadius;
                        if (difference < 0)
                            _verletVertices[i]._position -= positionDirection * difference;
                    }
                }

                // O(n²)
                // TODO: quadtree
                void SolveCollisions()
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                    {
                        VerletVertex currentVerletVertex = _verletVertices[i];
                        for (int j = 0; j < _verletVertices.Count; j++)
                        {
                            if (i == j) continue;
                            VerletVertex otherVerletVertex = _verletVertices[j];
                            currentVerletVertex.Collide(otherVerletVertex);
                        }
                    }
                }

                void SolveJoints()
                {
                    for (int i = 0; i < _verletEdges.Count; i++)
                        _verletEdges[i].Apply();
                }

                void UpdateVerlet(float deltaTime)
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                        _verletVertices[i].Update(deltaTime);
                }
            }

            public Verlet()
            {
                // data
                int screenWidth = 800;
                int screenHeight = 600;
                Vector2 screenCenter = new Vector2(screenWidth, screenHeight) / 2;
                double targetFPS = 120;

                Camera2D camera2D = new Camera2D(screenCenter, Vector2.Zero, 0, 1);
                VerletSolver verletSolver = new VerletSolver();

                Raylib.InitWindow(screenWidth, screenHeight, "Verlet");
                Raylib.SetTargetFPS((int)targetFPS);
                while (!Raylib.WindowShouldClose())
                {
                    float deltaTime = Raylib.GetFrameTime();
                    Vector2 mouseDelta = Raylib.GetMouseDelta();

                    // camera2d
                    camera2D.zoom += Raylib.GetMouseWheelMove() * 0.05f;
                    camera2D.zoom = Raylib_cs.Raymath.Clamp(camera2D.zoom, 0.1f, 3);
                    if (Raylib.IsMouseButtonDown(MouseButton.MOUSE_MIDDLE_BUTTON))
                        camera2D.target -= mouseDelta / camera2D.zoom;
                    Vector2 mouseScreenPos = Raylib.GetMousePosition();
                    Vector2 mouseWorldPos = Raylib.GetScreenToWorld2D(mouseScreenPos, camera2D);

                    // create Verlet
                    if (Raylib.IsMouseButtonPressed(MouseButton.MOUSE_BUTTON_LEFT))
                    {
                        // CreateStrip(verletSolver, mouseWorldPos, mouseDelta);
                        CreateGrid(verletSolver, mouseWorldPos, mouseDelta);
                    }

                    // render
                    Raylib.BeginDrawing();
                    Raylib.ClearBackground(Color.RAYWHITE);

                    Raylib.BeginMode2D(camera2D);
                    verletSolver.Solve(deltaTime);
                    verletSolver.Render();
                    Raylib.DrawLine(short.MinValue, 0, short.MaxValue, 0, Color.RED);
                    Raylib.DrawLine(0, short.MinValue, 0, short.MaxValue, Color.GREEN);
                    Raylib.EndMode2D();

                    Raylib.DrawText($"deltaTime:{deltaTime}", 12, 12, 20, Color.BLACK);
                    Raylib.EndDrawing();
                }
                Raylib.CloseWindow();
            }

            void CreateStrip(VerletSolver solver, Vector2 positionWS, Vector2 force)
            {
                const int linkSize = 20;
                float randMass = 1;
                float randRadius = Raylib.GetRandomValue(1, 5);
                VerletVertex[] vertices = new VerletVertex[linkSize];
                for (int i = 0; i < vertices.Length; i++)
                {
                    vertices[i] = new VerletVertex(positionWS + new Vector2(2, 0) * randRadius * i, randMass * randRadius, randRadius);
                    vertices[i].AddForce(force);
                    solver._verletVertices.Add(vertices[i]);
                }
                for (int i = 0; i < vertices.Length; i++)
                {
                    if (i < vertices.Length - 1)
                    {
                        VerletEdge edge = new VerletEdge(vertices[i], vertices[i + 1]);
                        solver._verletEdges.Add(edge);
                    }
                    // else
                    // {
                    //     VerletEdge edge = new VerletEdge(vertices[vertices.Length - 1], vertices[0]);
                    //     solver._verletEdges.Add(edge);
                    // }
                }
            }

            void CreateGrid(VerletSolver solver, Vector2 positionWS, Vector2 force)
            {
                const int gridSizeX = 5;
                const int gridSizeY = 5;
                VerletVertex[,] vertices = new VerletVertex[gridSizeX, gridSizeY];

                for (int x = 0; x < gridSizeX; x++)
                    for (int y = 0; y < gridSizeY; y++)
                    {
                        VerletVertex vertex = new VerletVertex(positionWS + new Vector2(x, y) * 4, 1, 2);
                        vertex.AddForce(force);
                        vertices[x, y] = vertex;
                        solver._verletVertices.Add(vertex);
                    }

                for (int x = 0; x < gridSizeX; x++)
                    for (int y = 0; y < gridSizeX; y++)
                    {
                        VerletVertex vertex = vertices[x, y];

                        int downIndex = y + 1;
                        if (downIndex > -1 && downIndex < gridSizeY)
                        {
                            VerletEdge edge = new VerletEdge(vertex, vertices[x, downIndex]);
                            solver._verletEdges.Add(edge);
                        }

                        int rightIndex = x + 1;
                        if (rightIndex > -1 && rightIndex < gridSizeX)
                        {
                            VerletEdge edge = new VerletEdge(vertex, vertices[rightIndex, y]);
                            solver._verletEdges.Add(edge);
                        }
                    }
            }
        }
    }
}