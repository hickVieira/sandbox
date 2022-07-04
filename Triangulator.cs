using System;
using System.Collections.Generic;
using System.Numerics;
using Raylib_cs;

namespace Sandbox
{
    public static partial class Programs
    {
        public class Triangulator
        {
            public struct vec2Sharp : DelaunatorSharp.IPoint
            {
                public double X { get; set; }
                public double Y { get; set; }
            }

            public struct vec2QHull : MIConvexHull.IVertex
            {
                public double[] Position { get; set; }
                public double x { get => Position[0]; set => Position[0] = value; }
                public double y { get => Position[1]; set => Position[1] = value; }
                public vec2QHull(double x, double y) { Position = new double[] { x, y }; }
            }

            public class triangleQHull : MIConvexHull.TriangulationCell<vec2QHull, triangleQHull>
            {
            }

            // data
            int _screenWidth = 800;
            int _screenHeight = 600;
            double _targetFPS = 120;
            float _mouseDeltaForce = 100;
            public Triangulator()
            {
                Vector2 screenCenter = new Vector2(_screenWidth, _screenHeight) / 2;
                Camera2D camera2D = new Camera2D(screenCenter, Vector2.Zero, 0, 1);
                List<Vector2> points = new List<Vector2>();

                Raylib.InitWindow(_screenWidth, _screenHeight, "Triangulator");
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

                    // create vertex
                    if (Raylib.IsMouseButtonPressed(MouseButton.MOUSE_BUTTON_LEFT) || Raylib.IsMouseButtonDown(MouseButton.MOUSE_BUTTON_RIGHT))
                        points.Add(mouseWorldPos);

                    if (Raylib.IsKeyPressed(KeyboardKey.KEY_R))
                        points.Clear();

                    System.Diagnostics.Stopwatch delaunayNet = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch delaunaySharp = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch delaunaySHull = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch delaunayQHull = new System.Diagnostics.Stopwatch();

                    DelaunatorNet.TriangulationInfo netTriangleInfo = null;
                    int[] sharpTriangles = null;
                    DelaunatorSharp.IPoint[] sharpPoints = null;
                    List<DelaunayTriangulator.Triad> shullTriads = null;
                    List<DelaunayTriangulator.Vertex> shullPoints = null;
                    IEnumerable<triangleQHull> qhullTriangles = null;
                    if (points.Count > 2)
                    {
                        {
                            double[] vertices = new double[points.Count * 2];
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                vertices[2 * i + 0] = vec.X;
                                vertices[2 * i + 1] = vec.Y;
                            }
                            delaunayNet.Start();
                            DelaunatorNet.Delaunator delaunator = new DelaunatorNet.Delaunator(vertices);
                            netTriangleInfo = delaunator.Build();
                            delaunayNet.Stop();
                        }
                        {
                            DelaunatorSharp.IPoint[] vertices = new DelaunatorSharp.IPoint[points.Count];
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                vertices[i] = new vec2Sharp() { X = vec.X, Y = vec.Y };
                            }
                            delaunaySharp.Start();
                            DelaunatorSharp.Delaunator delaunator = new DelaunatorSharp.Delaunator(vertices);
                            sharpTriangles = delaunator.Triangles;
                            sharpPoints = delaunator.Points;
                            delaunaySharp.Stop();
                        }
                        {
                            shullPoints = new List<DelaunayTriangulator.Vertex>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                shullPoints.Add(new DelaunayTriangulator.Vertex(vec.X, vec.Y));
                            }
                            delaunaySHull.Start();
                            DelaunayTriangulator.Triangulator triangulator = new DelaunayTriangulator.Triangulator();
                            shullTriads = triangulator.Triangulation(shullPoints, true);
                            delaunaySHull.Stop();
                        }
                        if (points.Count > 3)
                        {
                            List<vec2QHull> vertices = new List<vec2QHull>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                vertices.Add(new vec2QHull(vec.X, vec.Y));
                            }
                            delaunayQHull.Start();
                            qhullTriangles = MIConvexHull.Triangulation.CreateDelaunay<vec2QHull, triangleQHull>(vertices).Cells;
                            delaunayQHull.Stop();
                        }
                    }

                    // render
                    Raylib.BeginDrawing();
                    Raylib.ClearBackground(Color.RAYWHITE);

                    Raylib.BeginMode2D(camera2D);
                    if (netTriangleInfo != null)
                    {
                        for (int i = 0; i < netTriangleInfo.Triangles.Length; i += 3)
                        {
                            int vIndex0 = netTriangleInfo.Triangles[i + 0] * 2;
                            int vIndex1 = netTriangleInfo.Triangles[i + 1] * 2;
                            int vIndex2 = netTriangleInfo.Triangles[i + 2] * 2;
                            Vector2 v0 = new Vector2((float)netTriangleInfo.Points[vIndex0 + 0], (float)netTriangleInfo.Points[vIndex0 + 1]);
                            Vector2 v1 = new Vector2((float)netTriangleInfo.Points[vIndex1 + 0], (float)netTriangleInfo.Points[vIndex1 + 1]);
                            Vector2 v2 = new Vector2((float)netTriangleInfo.Points[vIndex2 + 0], (float)netTriangleInfo.Points[vIndex2 + 1]);
                            Raylib.DrawTriangleLines(v0, v1, v2, Color.RED);
                        }
                    }
                    if (sharpTriangles != null)
                    {
                        for (int i = 0; i < sharpTriangles.Length; i += 3)
                        {
                            int vIndex0 = sharpTriangles[i + 0];
                            int vIndex1 = sharpTriangles[i + 1];
                            int vIndex2 = sharpTriangles[i + 2];
                            Raylib.DrawTriangleLines(points[vIndex0], points[vIndex1], points[vIndex2], Color.BLUE);
                        }
                    }
                    if (shullTriads != null)
                    {
                        for (int i = 0; i < shullTriads.Count; i++)
                        {
                            DelaunayTriangulator.Triad triad = shullTriads[i];
                            Vector2 v0 = new Vector2(shullPoints[triad.a].x, shullPoints[triad.a].y);
                            Vector2 v1 = new Vector2(shullPoints[triad.b].x, shullPoints[triad.b].y);
                            Vector2 v2 = new Vector2(shullPoints[triad.c].x, shullPoints[triad.c].y);
                            Raylib.DrawTriangleLines(v0, v1, v2, Color.GREEN);
                        }
                    }
                    if (qhullTriangles != null)
                    {
                        foreach(var triangle in qhullTriangles)
                        {
                            Vector2 v0 = new Vector2((float)triangle.Vertices[0].x, (float)triangle.Vertices[0].y);
                            Vector2 v1 = new Vector2((float)triangle.Vertices[1].x, (float)triangle.Vertices[1].y);
                            Vector2 v2 = new Vector2((float)triangle.Vertices[2].x, (float)triangle.Vertices[2].y);
                            Raylib.DrawTriangleLines(v0, v1, v2, Color.BLACK);
                        }
                    }
                    Raylib.DrawLine(short.MinValue, 0, short.MaxValue, 0, Color.RED);
                    Raylib.DrawLine(0, short.MinValue, 0, short.MaxValue, Color.GREEN);
                    Raylib.EndMode2D();

                    Raylib.DrawText($"delaunayNet:{delaunayNet.Elapsed.TotalMilliseconds}", 12, 40 + 1, 20, Color.RED);
                    Raylib.DrawText($"delaunaySharp:{delaunaySharp.Elapsed.TotalMilliseconds}", 12, 60 + 1, 20, Color.BLUE);
                    Raylib.DrawText($"S-hull:{delaunaySHull.Elapsed.TotalMilliseconds}", 12, 80 + 1, 20, Color.GREEN);
                    Raylib.DrawText($"MIConvexHull:{delaunayQHull.Elapsed.TotalMilliseconds}", 12, 100 + 1, 20, Color.BLACK);
                    Raylib.EndDrawing();
                }
                Raylib.CloseWindow();
            }
        }
    }
}