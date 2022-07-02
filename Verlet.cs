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
                public float _mass;
                public float _radius;
                private Vector2 _position;
                private Vector2 _position_old;
                private Vector2 _acceleration;
                private Vector2 _force;

                public Vector2 position { get => _position; }
                public Vector2 velocity { get => _position - _position_old; }

                public VerletVertex(Vector2 position, float mass, float radius)
                {
                    this._position = position;
                    this._mass = mass;
                    this._radius = radius;
                    this._position_old = position;
                    this._acceleration = Vector2.Zero;
                    this._force = Vector2.Zero;
                }

                public void Render(Color color)
                {
                    Raylib.DrawCircle((int)_position.X, (int)_position.Y, _radius, color);
                }

                public void Update(float deltaTime)
                {
                    Vector2 currentVelocity = velocity;
                    _position_old = _position;
                    _position = (_position + currentVelocity) + (_force * deltaTime / _mass) + (_acceleration * deltaTime * deltaTime);
                    _acceleration = Vector2.Zero;
                    _force = Vector2.Zero;
                }

                public void Displace(Vector2 offset)
                {
                    _position += offset;
                }

                public void Stop()
                {
                    _position_old = _position;
                }

                public void AddAcceleration(Vector2 accel)
                {
                    _acceleration += accel;
                }

                public void AddForce(Vector2 force)
                {
                    _force += force;
                }

                public void Move(Vector2 offset)
                {
                    _position += offset;
                    _position_old += offset;
                }

                public void Collide(VerletVertex otherVerletVertex)
                {
                    Vector2 collisionVector = _position - otherVerletVertex.position;
                    float collisionLength = collisionVector.Length();
                    float collisionDistance = collisionLength - _radius - otherVerletVertex._radius;
                    if (collisionDistance < 0)
                    {
                        float penetration = System.Math.Abs(collisionDistance);
                        Vector2 normal = collisionVector / collisionLength;
                        _position += normal * penetration / (1 + _mass);
                        otherVerletVertex._position -= normal * penetration / (1 + otherVerletVertex._mass);
                    }
                }
            }

            public class VerletEdge
            {
                public VerletVertex _verletVertex0;
                public VerletVertex _verletVertex1;
                public float _distance;
                public float _breakDistance;
                public bool _isBroken;

                public VerletEdge(VerletVertex verletVertex0, VerletVertex verletVertex1, float breakDistance = 20)
                {
                    this._verletVertex0 = verletVertex0;
                    this._verletVertex1 = verletVertex1;
                    this._distance = Vector2.Distance(verletVertex0.position, verletVertex1.position) + float.Epsilon;
                    this._breakDistance = _distance + breakDistance + float.Epsilon;
                }

                public void Render()
                {
                    Raylib.DrawLine((int)_verletVertex0.position.X, (int)_verletVertex0.position.Y, (int)_verletVertex1.position.X, (int)_verletVertex1.position.Y, Color.GRAY);
                }

                public void Apply()
                {
                    Vector2 edgeVector = _verletVertex0.position - _verletVertex1.position;
                    float edgeLength = edgeVector.Length();
                    float edgeDistance = _distance - edgeLength;

                    if (edgeLength > _breakDistance)
                        _isBroken = true;

                    Vector2 normal = edgeVector / edgeLength;
                    _verletVertex0.Displace(+0.5f * normal * edgeDistance);
                    _verletVertex1.Displace(-0.5f * normal * edgeDistance);
                }

                public void Apply(float springK, float springDampness = 0.01f)
                {
                    Vector2 edgeVector = _verletVertex0.position - _verletVertex1.position;
                    float edgeLength = edgeVector.Length();
                    float edgeDistance = _distance - edgeLength;

                    if (edgeLength > _breakDistance)
                        _isBroken = true;

                    Vector2 normal = edgeVector / edgeLength;

                    float dampDot = Vector2.Dot(edgeVector, _verletVertex0.velocity - _verletVertex1.velocity);
                    Vector2 force0 = +0.5f * springK * edgeDistance * normal - springDampness * dampDot * normal;
                    Vector2 force1 = -0.5f * springK * edgeDistance * normal + springDampness * dampDot * normal;

                    _verletVertex0.AddForce(force0);
                    _verletVertex1.AddForce(force1);
                }
            }

            public class VerletSolver
            {
                public List<VerletVertex> _verletVertices;
                public List<VerletEdge> _verletEdges;
                public QuadTree<VerletVertex> _quadTree;

                const uint _subStepsCount = 4;
                const float _boundaryRadius = 200;
                // const float _gravityForce = 9.81f;
                // const float _gravityForce = 9.81f * 5520;
                const float _gravityForce = 9.81f * 10;

                public VerletSolver()
                {
                    _verletVertices = new List<VerletVertex>();
                    _verletEdges = new List<VerletEdge>();
                }

                public void Render()
                {
                    Raylib.DrawCircle(0, 0, _boundaryRadius, Color.WHITE);
                    for (int i = 0; i < _verletVertices.Count; i++)
                        _verletVertices[i].Render(Color.BLACK);
                    for (int i = 0; i < _verletEdges.Count; i++)
                        _verletEdges[i].Render();
                    _quadTree.Draw(Color.LIME);
                }

                public void Solve(float deltaTime)
                {
                    SolveQuadTree(); // ideally it should called somewhere before collision solving
                    float subDeltaTime = deltaTime / _subStepsCount;
                    for (int i = 0; i < _subStepsCount; i++)
                    {
                        ApplyAcceleration();
                        SolveBoundaries();
                        SolveEdges();
                        SolveCollisions();
                        UpdateVerlet(subDeltaTime);
                    }
                }

                void SolveQuadTree()
                {
                    _quadTree = new QuadTree<VerletVertex>(Vector2.Zero, Vector2.One * 3200, 7);
                    for (int i = 0; i < _verletVertices.Count; i++)
                    {
                        VerletVertex vertex = _verletVertices[i];
                        Vector2 boundsVector = Vector2.One * (vertex.velocity.Length() + vertex._radius * 4);
                        _quadTree.Insert(vertex, new AABB(vertex.position, boundsVector));
                    }
                }

                void ApplyAcceleration()
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                    {
                        VerletVertex verletVertex = _verletVertices[i];
                        Vector2 force = (verletVertex._mass * new Vector2(0, 1) * _gravityForce);
                        _verletVertices[i].AddAcceleration(force);
                    }
                    // for (int i = 0; i < _verletVertices.Count; i++)
                    // {
                    //     VerletVertex verletVertex = _verletVertices[i];
                    //     Vector2 toCenterVector = Vector2.Zero - verletVertex.position;
                    //     float toCenterLength = toCenterVector.Length();
                    //     Vector2 toCenterDireciton = toCenterVector / toCenterLength;
                    //     Vector2 force = (verletVertex._mass * toCenterDireciton * _gravityForce * _boundaryRadius * _gravityDensity) / (toCenterLength * toCenterLength);
                    //     verletVertex.AddAcceleration(force);
                    // }
                }

                void SolveBoundaries()
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                    {
                        float positionLength = _verletVertices[i].position.Length();
                        Vector2 positionDirection = _verletVertices[i].position / positionLength;
                        float difference = (positionLength + _verletVertices[i]._radius) - _boundaryRadius;
                        if (difference > 0)
                            _verletVertices[i].Displace(-positionDirection * difference);
                    }
                }

                void SolveCollisions()
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                    {
                        VerletVertex currentVerletVertex = _verletVertices[i];

                        var nodeIndex = _quadTree.GetNodeIndex(currentVerletVertex.position, 0);
                        if (nodeIndex == -1) continue;

                        List<VerletVertex> nodeItems = _quadTree.GetNodeItems(nodeIndex);
                        if (nodeItems == null)
                            continue;

                        for (int j = 0; j < nodeItems.Count; j++)
                        {
                            VerletVertex otherVerletVertex = nodeItems[j];
                            if (object.ReferenceEquals(currentVerletVertex, otherVerletVertex)) continue;
                            currentVerletVertex.Collide(otherVerletVertex);
                        }
                    }
                }

                void SolveEdges()
                {
                    for (int i = 0; i < _verletEdges.Count; i++)
                    {
                        VerletEdge edge = _verletEdges[i];
                        if (edge._isBroken)
                            _verletEdges.RemoveAt(i--);
                        else
                            edge.Apply(5, 1f);
                    }
                }

                void UpdateVerlet(float deltaTime)
                {
                    for (int i = 0; i < _verletVertices.Count; i++)
                        _verletVertices[i].Update(deltaTime);
                }
            }

            // data
            int _screenWidth = 800;
            int _screenHeight = 600;
            double _targetFPS = 120;
            float _mouseDeltaForce = 100;
            public Verlet()
            {
                Vector2 screenCenter = new Vector2(_screenWidth, _screenHeight) / 2;
                Camera2D camera2D = new Camera2D(screenCenter, Vector2.Zero, 0, 1);
                VerletSolver verletSolver = new VerletSolver();

                Raylib.SetConfigFlags(ConfigFlags.FLAG_MSAA_4X_HINT | ConfigFlags.FLAG_WINDOW_HIGHDPI);
                Raylib.InitWindow(_screenWidth, _screenHeight, "Verlet");
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

                    // create Verlet
                    if (Raylib.IsMouseButtonPressed(MouseButton.MOUSE_BUTTON_LEFT))
                    // if (Raylib.IsMouseButtonDown(MouseButton.MOUSE_BUTTON_LEFT))
                    {
                        // CreateVerletVertex(verletSolver, mouseWorldPos, mouseDelta);
                        // CreateStrip(verletSolver, mouseWorldPos, mouseDelta);
                        CreateGrid(verletSolver, mouseWorldPos, mouseDelta);
                    }

                    if (Raylib.IsKeyPressed(KeyboardKey.KEY_R))
                        verletSolver = new VerletSolver();

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

            void CreateVerletVertex(VerletSolver solver, Vector2 positionWS, Vector2 force)
            {
                float randMass = 1;
                float randRadius = 4;
                solver._verletVertices.Add(new VerletVertex(positionWS, randMass * randRadius, randRadius));
            }

            void CreateStrip(VerletSolver solver, Vector2 positionWS, Vector2 force)
            {
                const int edgeSize = 20;
                float randMass = 1;
                float randRadius = Raylib.GetRandomValue(1, 5);
                VerletVertex[] vertices = new VerletVertex[edgeSize];
                for (int i = 0; i < vertices.Length; i++)
                {
                    vertices[i] = new VerletVertex(positionWS + 2 * new Vector2(1, 0) * randRadius * i, randMass * randRadius, randRadius);
                    vertices[i].AddForce(force * _mouseDeltaForce);
                    solver._verletVertices.Add(vertices[i]);
                }
                for (int i = 0; i < vertices.Length; i++)
                {
                    if (i < vertices.Length - 1)
                    {
                        VerletEdge edge = new VerletEdge(vertices[i], vertices[i + 1]);
                        solver._verletEdges.Add(edge);
                    }
                }
            }

            void CreateGrid(VerletSolver solver, Vector2 positionWS, Vector2 force)
            {
                const float ballSize = 4;
                const int gridSizeX = 5;
                const int gridSizeY = 5;
                VerletVertex[,] vertices = new VerletVertex[gridSizeX, gridSizeY];

                for (int x = 0; x < gridSizeX; x++)
                    for (int y = 0; y < gridSizeY; y++)
                    {
                        VerletVertex vertex = new VerletVertex(positionWS + 4 * ballSize * new Vector2(x, y), 1, ballSize);
                        vertex.AddForce(force * _mouseDeltaForce);
                        vertices[x, y] = vertex;
                        solver._verletVertices.Add(vertex);
                    }

                for (int x = 0; x < gridSizeX; x++)
                    for (int y = 0; y < gridSizeX; y++)
                    {
                        VerletVertex vertex = vertices[x, y];

                        // right
                        if ((x + 1) > -1 && (x + 1) < gridSizeX)
                            solver._verletEdges.Add(new VerletEdge(vertex, vertices[(x + 1), y]));

                        // down
                        if ((y + 1) > -1 && (y + 1) < gridSizeY)
                            solver._verletEdges.Add(new VerletEdge(vertex, vertices[x, (y + 1)]));

                        // right_down
                        if (((x + 1) > -1 && (x + 1) < gridSizeX) && ((y + 1) > -1 && (y + 1) < gridSizeY))
                            solver._verletEdges.Add(new VerletEdge(vertex, vertices[(x + 1), (y + 1)]));

                        // left_down
                        if (((x - 1) > -1 && (x - 1) < gridSizeX) && ((y + 1) > -1 && (y + 1) < gridSizeY))
                            solver._verletEdges.Add(new VerletEdge(vertex, vertices[(x - 1), (y + 1)]));
                    }
            }
        }
    }
}