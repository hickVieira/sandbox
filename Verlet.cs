using System.Collections.Generic;
using System.Numerics;
using Raylib_cs;

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
                    this._distance = Vector2.Distance(verletVertex0._position, verletVertex1._position) + (verletVertex0._radius + verletVertex1._radius) / 2 + float.Epsilon;
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
                public Vector2 _gravity = new Vector2(0, 100);
                public float _boundaryRadius = 50;
                public uint _subStepsCount = 4;

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
                        UpdateVerletVertices(subDeltaTime);
                    }
                }

                void ApplyAcceleration(Vector2 accel)
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                        _verletVertices[i].AddAcceleration(accel);
                }

                void ApplyCenterGravity()
                {
                    const float gravityForce = 9.81f;
                    const float gravityDensity = 5520;
                    for (int i = 0; i < _verletVertices.Count; i++)
                    {
                        VerletVertex verletVertex = _verletVertices[i];
                        Vector2 toCenterVector = Vector2.Zero - verletVertex._position;
                        float toCenterLength = toCenterVector.Length();
                        Vector2 toCenterDireciton = toCenterVector / toCenterLength;
                        Vector2 force = (verletVertex._mass * toCenterDireciton * gravityForce * _boundaryRadius * gravityDensity) / (toCenterLength * toCenterLength);
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

                void UpdateVerletVertices(float deltaTime)
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
                const int linkSize = 1;

                Raylib.InitWindow(screenWidth, screenHeight, "Verlet");
                Raylib.SetTargetFPS((int)targetFPS);
                while (!Raylib.WindowShouldClose())
                {
                    float deltaTime = Raylib.GetFrameTime();

                    // camera2d
                    camera2D.zoom += Raylib.GetMouseWheelMove() * 0.01f;
                    camera2D.zoom = Raylib_cs.Raymath.Clamp(camera2D.zoom, 0, 3);
                    Vector2 mousePos = Raylib.GetMousePosition() - screenCenter;

                    // create Verlet
                    if (Raylib.IsMouseButtonDown(MouseButton.MOUSE_BUTTON_LEFT))
                    {
                        float randMass = 1;
                        float randRadius = Raylib.GetRandomValue(1, 5);
                        VerletVertex[] vertices = new VerletVertex[linkSize];
                        VerletEdge[] edges = new VerletEdge[linkSize];
                        for (int i = 0; i < vertices.Length; i++)
                        {
                            vertices[i] = new VerletVertex(mousePos + new Vector2(1, 0) * randRadius * i, randMass * randRadius, randRadius);
                            vertices[i].AddForce(new Vector2(Raylib.GetRandomValue(-1, 1), Raylib.GetRandomValue(-1, 1)));
                            verletSolver._verletVertices.Add(vertices[i]);
                        }
                        for (int i = 0; i < vertices.Length; i++)
                        {
                            if (i < vertices.Length - 1)
                            {
                                edges[i] = new VerletEdge(vertices[i], vertices[i + 1]);
                                verletSolver._verletEdges.Add(edges[i]);
                            }
                        }
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
        }
    }
}