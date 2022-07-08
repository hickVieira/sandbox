using System;
using System.Collections.Generic;
using System.Numerics;
using Raylib_cs;

namespace Sandbox
{
    public static partial class Programs
    {
        // ten minute physics pbd tutorials
        public class PBD
        {
            public class PBDParticle
            {
                public Vector2 _position;
                public Vector2 _velocity;
                public float _mass;
                public float _radius;
                public float _restitution;

                public PBDParticle(Vector2 position, float mass, float radius, float restitution = 0.5f)
                {
                    _position = position;
                    _velocity = Vector2.Zero;
                    _mass = mass;
                    _radius = radius;
                    _restitution = restitution;
                }

                public void AddForce(Vector2 force, float deltaTime)
                {
                    _velocity = _velocity + force * deltaTime;
                }

                public void Solve(float deltaTime)
                {
                    _position = _position + _velocity * deltaTime;
                }

                public void Collide(PBDParticle other)
                {
                    Vector2 collisionVector = _position - other._position;
                    float collisionLength = collisionVector.Length();

                    if (collisionLength == 0 || collisionLength > _radius + other._radius)
                        return;

                    float collisionDistance = _radius + other._radius - collisionLength;
                    Vector2 collisionNormal = collisionVector / collisionLength;
                    _position += collisionNormal * collisionDistance / 2;
                    other._position -= collisionNormal * collisionDistance / 2;

                    float v1 = Vector2.Dot(_velocity, collisionNormal);
                    float v2 = Vector2.Dot(other._velocity, collisionNormal);
                    float m1 = _mass;
                    float m2 = other._mass;
                    float restitution = (_restitution + other._restitution) / 2;

                    float m1v1m2v2 = m1 * v1 + m2 * v2;
                    float m1m2 = m1 + m2;
                    float newV1 = (m1v1m2v2 - m2 * (v1 - v2) * restitution) / m1m2;
                    float newV2 = (m1v1m2v2 - m1 * (v2 - v1) * restitution) / m1m2;

                    _velocity += collisionNormal * (newV1 - v1);
                    other._velocity += collisionNormal * (newV2 - v2);
                }
            }

            public class PBDSolver
            {
                public List<PBDParticle> _particles;
                public QuadTree<PBDParticle> _quadTree;

                private int _subStepCount = 32;
                private float _gravityForce = -10 * 100;
                private float _boundarySize = 200;

                public PBDSolver()
                {
                    _particles = new List<PBDParticle>();
                }

                public void Clear()
                {
                    _particles?.Clear();
                    _quadTree = null;
                }

                public void Render()
                {
                    Raylib.DrawRectangle(-(int)_boundarySize, -short.MaxValue, (int)_boundarySize * 2, short.MaxValue, Color.RAYWHITE);
                    _quadTree?.Draw(Color.LIME);
                    for (int i = 0; i < _particles.Count; i++)
                    {
                        PBDParticle particle = _particles[i];
                        Raylib.DrawCircle((int)particle._position.X, (int)particle._position.Y, particle._radius, Color.BLACK);
                    }
                }

                public void Solve(float deltaTime)
                {
                    deltaTime = deltaTime / _subStepCount;
                    for (int i = 0; i < _subStepCount; i++)
                    {
                        SolveGravityForce(deltaTime);
                        SolveBoundaries();
                        SolveQuadTree();
                        SolveCollisions();
                        UpdateParticles(deltaTime);
                    }
                }

                void SolveQuadTree()
                {
                    if (_quadTree == null) _quadTree = new QuadTree<PBDParticle>();
                    _quadTree.Update(Vector2.Zero, 3200 * Vector2.One, 7);

                    for (int i = 0; i < _particles.Count; i++)
                    {
                        PBDParticle particle = _particles[i];
                        if (particle == null)
                        {
                            _particles.RemoveAt(i--);
                            continue;
                        }
                        Vector2 boundsVector = Vector2.One * particle._radius * 2f;
                        _quadTree.Insert(particle, new AABB2(particle._position, boundsVector));
                    }
                }

                void SolveGravityForce(float deltaTime)
                {
                    for (int i = 0; i < _particles.Count; i++)
                    {
                        PBDParticle particle = _particles[i];
                        Vector2 force = (_gravityForce * new Vector2(0, -1));
                        particle.AddForce(force, deltaTime);
                    }
                }

                void SolveBoundaries()
                {
                    for (int i = 0; i < _particles.Count; i++)
                    {
                        PBDParticle particle = _particles[i];
                        if (particle._position.X < -_boundarySize + particle._radius)
                        {
                            particle._velocity.X = Math.Abs(particle._velocity.X);
                            particle._position.X = -_boundarySize + particle._radius;
                        }
                        if (particle._position.X > _boundarySize - particle._radius)
                        {
                            particle._velocity.X = -Math.Abs(particle._velocity.X);
                            particle._position.X = _boundarySize - particle._radius;
                        }
                        if (particle._position.Y > 0 - particle._radius)
                        {
                            particle._velocity.Y = -Math.Abs(particle._velocity.Y);
                            particle._position.Y = 0 - particle._radius;
                        }
                    }
                }

                void SolveCollisions()
                {
                    for (int i = 0; i < _particles.Count; i++)
                    {
                        PBDParticle particle = _particles[i];

                        var nodeIndex = _quadTree.GetNodeIndex(particle._position, 0);
                        if (nodeIndex == -1) continue;

                        List<PBDParticle> nodeItems = _quadTree.GetNodeItems(nodeIndex);
                        if (nodeItems == null) continue;

                        for (int j = 0; j < nodeItems.Count; j++)
                        {
                            PBDParticle otherParticle = nodeItems[j];
                            if (object.ReferenceEquals(particle, otherParticle)) continue;
                            particle.Collide(otherParticle);
                        }
                    }
                }

                void UpdateParticles(float deltaTime)
                {
                    for (int i = 0; i < _particles.Count; i++)
                        _particles[i].Solve(deltaTime);
                }
            }

            public PBD()
            {
                // data
                int _screenWidth = 800;
                int _screenHeight = 600;
                int _targetFPS = 60;

                Vector2 screenCenter = new Vector2(_screenWidth, _screenHeight) / 2;
                Camera2D camera2D = new Camera2D(screenCenter, Vector2.Zero, 0, 1);
                PBDSolver pbdSolver = new PBDSolver();

                Raylib.InitWindow(_screenWidth, _screenHeight, "PositionBasedDynamics");
                Raylib.SetTargetFPS((int)_targetFPS);
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

                    // create objects
                    if (Raylib.IsMouseButtonPressed(MouseButton.MOUSE_BUTTON_LEFT) || Raylib.IsMouseButtonDown(MouseButton.MOUSE_BUTTON_RIGHT))
                    {
                        PBDParticle newParticle = new PBDParticle(mouseWorldPos, 1, 10);
                        newParticle.AddForce(mouseDelta * 100, deltaTime);
                        pbdSolver._particles.Add(newParticle);
                    }

                    if (Raylib.IsKeyPressed(KeyboardKey.KEY_R))
                        pbdSolver.Clear();

                    pbdSolver.Solve(deltaTime);

                    // render
                    Raylib.BeginDrawing();
                    {
                        Raylib.ClearBackground(Color.BLACK);

                        Raylib.BeginMode2D(camera2D);
                        {
                            pbdSolver.Render();
                            Raylib.DrawLine(short.MinValue, 0, short.MaxValue, 0, Color.RED);
                            Raylib.DrawLine(0, short.MinValue, 0, short.MaxValue, Color.GREEN);
                        }
                        Raylib.EndMode2D();

                        Raylib.DrawText($"deltaTime:{deltaTime}", 12, 20 + 1, 20, Color.WHITE);
                    }
                    Raylib.EndDrawing();
                }
                Raylib.CloseWindow();
            }
        }
    }
}