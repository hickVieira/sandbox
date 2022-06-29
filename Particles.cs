using System.Collections.Generic;
using System.Numerics;
using Raylib_cs;

namespace Sandbox
{
    public static partial class Programs
    {
        public class Particles
        {
            class Particle
            {
                public float _radius;
                public Vector2 _position;
                public Vector2 _velocity;

                public Particle(float radius, Vector2 position, Vector2 velocity)
                {
                    this._radius = radius;
                    this._position = position;
                    this._velocity = velocity;
                }

                public void Render()
                {
                    Raylib.DrawCircle((int)_position.X, (int)_position.Y, _radius, Color.BLACK);
                }

                public void Update(float deltaTime)
                {
                    _position += _velocity * deltaTime;
                    _velocity -= _velocity * deltaTime * 0.1f;
                }
            }
            List<Particle> _particles;

            public Particles()
            {
                // data
                int screenWidth = 800;
                int screenHeight = 600;
                Vector2 screenCenter = new Vector2(screenWidth, screenHeight) / 2;
                double targetFPS = 60;

                Camera2D camera2D = new Camera2D(screenCenter, screenCenter, 0, 1);
                _particles = new List<Particle>();
                float gravityForce = 1000;

                Raylib.SetConfigFlags(ConfigFlags.FLAG_MSAA_4X_HINT);
                Raylib.InitWindow(screenWidth, screenHeight, "Particles");
                Raylib.SetTargetFPS((int)targetFPS);
                while (!Raylib.WindowShouldClose())
                {
                    float deltaTime = Raylib.GetFrameTime();

                    // camera2d
                    camera2D.zoom += Raylib.GetMouseWheelMove() * 0.01f;
                    camera2D.zoom = Raylib_cs.Raymath.Clamp(camera2D.zoom, 0, 3);
                    Vector2 mousePos = Raylib.GetMousePosition();

                    // create particles
                    if (Raylib.IsMouseButtonDown(MouseButton.MOUSE_BUTTON_LEFT))
                    {
                        int particlesToCreate = (int)(1.0 / deltaTime);
                        for (int i = 0; i < particlesToCreate; i++)
                        {
                            float randRadius = Raylib.GetRandomValue(1, 2);
                            Vector2 randPosition = mousePos + new Vector2(Raylib.GetRandomValue(-50, 50), Raylib.GetRandomValue(-50, 50));
                            Vector2 randVelocity = new Vector2(Raylib.GetRandomValue(-100, 100), Raylib.GetRandomValue(-100, 100));
                            _particles.Add(new Particle(randRadius, randPosition, randVelocity));
                        }
                    }

                    // add forces
                    if (_particles.Count > 0)
                    {
                        Vector2 gravityCenter;
                        if (Raylib.IsMouseButtonDown(MouseButton.MOUSE_BUTTON_RIGHT))
                        {
                        gravityCenter = mousePos;
                        }
                        else
                        {
                            gravityCenter = _particles[0]._position;
                            for (int i = 1; i < _particles.Count; i++)
                                gravityCenter += _particles[i]._position;
                            gravityCenter /= _particles.Count;
                        }

                        for (int i = 0; i < _particles.Count; i++)
                        {
                            Vector2 toCenterVector = gravityCenter - _particles[i]._position;
                            float toCenterDistance = toCenterVector.Length();
                            Vector2 toCenterDirection = toCenterVector / (float.Epsilon + toCenterDistance);
                            Vector2 force = gravityForce * toCenterDirection / (100 + toCenterDistance);
                            _particles[i]._velocity += force;
                        }
                    }

                    // render
                    Raylib.BeginDrawing();
                    Raylib.ClearBackground(Color.RAYWHITE);

                    Raylib.BeginMode2D(camera2D);
                    for (int i = 0; i < _particles.Count; i++)
                    {
                        _particles[i].Update(deltaTime);
                        _particles[i].Render();
                    }
                    Raylib.EndMode2D();

                    Raylib.DrawText($"deltaTime:{deltaTime}", 12, 12, 20, Color.BLACK);
                    Raylib.EndDrawing();
                }
                Raylib.CloseWindow();
            }
        }
    }
}