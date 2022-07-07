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

            public Triangulator()
            {
                int _screenWidth = (int)(800);
                int _screenHeight = (int)(600);
                double _targetFPS = 120;
                Vector2 _screenCenter = new Vector2(_screenWidth, _screenHeight) / 2;
                Camera2D _camera2D = new Camera2D(_screenCenter, Vector2.Zero, 0, 1);
                List<Vector2> _points = new List<Vector2>();

                Raylib.InitWindow(_screenWidth, _screenHeight, "Triangulator");
                Raylib.SetTargetFPS((int)_targetFPS);
                while (!Raylib.WindowShouldClose())
                {
                    float deltaTime = Raylib.GetFrameTime();
                    Vector2 mouseDelta = Raylib.GetMouseDelta();

                    // camera2d
                    _camera2D.zoom += Raylib.GetMouseWheelMove() * 0.05f;
                    _camera2D.zoom = Raylib_cs.Raymath.Clamp(_camera2D.zoom, 0.025f, 3);
                    if (Raylib.IsMouseButtonDown(MouseButton.MOUSE_MIDDLE_BUTTON))
                        _camera2D.target -= mouseDelta / _camera2D.zoom;
                    Vector2 mouseScreenPos = Raylib.GetMousePosition();
                    Vector2 mouseWorldPos = Raylib.GetScreenToWorld2D(mouseScreenPos, _camera2D);

                    bool isClick = Raylib.IsMouseButtonPressed(MouseButton.MOUSE_BUTTON_LEFT);
                    bool isHold = Raylib.IsMouseButtonDown(MouseButton.MOUSE_BUTTON_RIGHT);

                    // create vertex
                    if (isClick)
                        _points.Add(mouseWorldPos);
                    if (isHold && _points.Count > 0 && Vector2.Distance(mouseWorldPos, _points[_points.Count - 1]) > 4)
                        _points.Add(mouseWorldPos);

                    if (Raylib.IsKeyPressed(KeyboardKey.KEY_R))
                        _points.Clear();

                    // render
                    Raylib.BeginDrawing();
                    {
                        Raylib.ClearBackground(Color.GRAY);

                        test_myDelaunator(_camera2D, 12, 40 + 1, 20, _points, Color.RED);
                        test_DelaunatorNet(_camera2D, 12, 60 + 1, 20, _points, Color.GREEN);
                        test_DelaunatorSharp(_camera2D, 12, 80 + 1, 20, _points, Color.BLUE);
                        test_DelaunatorCSharp(_camera2D, 12, 100 + 1, 20, _points, Color.GOLD);
                        test_SHull(_camera2D, 12, 120 + 1, 20, _points, Color.SKYBLUE);
                        test_MIConvexHull(_camera2D, 12, 140 + 1, 20, _points, Color.BLACK);
                        test_Poly2Tri(_camera2D, 12, 160 + 1, 20, _points, Color.YELLOW);
                        test_Poly2Trics(_camera2D, 12, 180 + 1, 20, _points, Color.PINK);
                        test_Poly2TriBiofluidix(_camera2D, 12, 200 + 1, 20, _points, Color.BEIGE);
                        test_LibTessDotNet(_camera2D, 12, 220 + 1, 20, _points, Color.RAYWHITE);
                        test_DelaunayOskar(_camera2D, 12, 240 + 1, 20, _points, Color.ORANGE);
                        test_QuickHull3DBiofluidix(_camera2D, 12, 260 + 1, 20, _points, Color.PURPLE);
                        test_QuickHull3DOskar(_camera2D, 12, 280 + 1, 20, _points, Color.WHITE);

                        Raylib.BeginMode2D(_camera2D);
                        {
                            Raylib.DrawLine(short.MinValue, 0, short.MaxValue, 0, Color.RED);
                            Raylib.DrawLine(0, short.MinValue, 0, short.MaxValue, Color.GREEN);
                            for (int i = 0; i < _points.Count; i++)
                                Raylib.DrawCircle((int)_points[i].X, (int)_points[i].Y, 1, Color.BLACK);
                        }
                        Raylib.EndMode2D();
                    }
                    Raylib.EndDrawing();
                }
                Raylib.CloseWindow();
            }

            void test_myDelaunator(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                List<decimal2> vertices = new List<decimal2>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices.Add(vec);
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                List<Delaunay.Triangle> triangles = Delaunay.BowyerWatson2D(vertices, 10000);
                stopwatch.Stop();

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    for (int i = 0; i < triangles.Count; i++)
                    {
                        int vIndex0 = triangles[i]._vIndex0;
                        int vIndex1 = triangles[i]._vIndex1;
                        int vIndex2 = triangles[i]._vIndex2;
                        Raylib.DrawTriangleLines((Vector2)vertices[vIndex0], (Vector2)vertices[vIndex1], (Vector2)vertices[vIndex2], color);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"myDelaunator:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_DelaunatorNet(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                double[] vertices = new double[inPoints.Count * 2];
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices[2 * i + 0] = vec.X;
                    vertices[2 * i + 1] = vec.Y;
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                DelaunatorNet.Delaunator delaunator = new DelaunatorNet.Delaunator(vertices);
                var netTriangleInfo = delaunator.Build();
                stopwatch.Stop();

                // draw
                Raylib.BeginMode2D(camera2D);
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
                Raylib.EndMode2D();
                Raylib.DrawText($"delaunator-net:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_DelaunatorSharp(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                DelaunatorSharp.IPoint[] vertices = new DelaunatorSharp.IPoint[inPoints.Count];
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices[i] = new vec2Sharp() { X = vec.X, Y = vec.Y };
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                DelaunatorSharp.Delaunator delaunator = new DelaunatorSharp.Delaunator(vertices);
                stopwatch.Stop();
                var sharpTriangles = delaunator.Triangles;
                var sharpPoints = delaunator.Points;

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    for (int i = 0; i < sharpTriangles.Length; i += 3)
                    {
                        int vIndex0 = sharpTriangles[i + 0];
                        int vIndex1 = sharpTriangles[i + 1];
                        int vIndex2 = sharpTriangles[i + 2];
                        Raylib.DrawTriangleLines(inPoints[vIndex0], inPoints[vIndex1], inPoints[vIndex2], Color.BLUE);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"delaunator-sharp:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_DelaunatorCSharp(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                var csharpVertices = new List<double>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    csharpVertices.Add(vec.X);
                    csharpVertices.Add(vec.Y);
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                Delaunator.Triangulation triangulation = new Delaunator.Triangulation(csharpVertices);
                stopwatch.Stop();
                var csharpTriangles = triangulation.triangles;

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    for (int i = 0; i < csharpTriangles.Count; i += 3)
                    {
                        int vIndex0 = csharpTriangles[i + 0] * 2;
                        int vIndex1 = csharpTriangles[i + 1] * 2;
                        int vIndex2 = csharpTriangles[i + 2] * 2;
                        Vector2 v0 = new Vector2((float)csharpVertices[vIndex0 + 0], (float)csharpVertices[vIndex0 + 1]);
                        Vector2 v1 = new Vector2((float)csharpVertices[vIndex1 + 0], (float)csharpVertices[vIndex1 + 1]);
                        Vector2 v2 = new Vector2((float)csharpVertices[vIndex2 + 0], (float)csharpVertices[vIndex2 + 1]);
                        Raylib.DrawTriangleLines(v0, v1, v2, Color.ORANGE);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"delaunator-csharp:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_SHull(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                var vertices = new List<DelaunayTriangulator.Vertex>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices.Add(new DelaunayTriangulator.Vertex(vec.X, vec.Y));
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                DelaunayTriangulator.Triangulator triangulator = new DelaunayTriangulator.Triangulator();
                var triads = triangulator.Triangulation(vertices, true);
                stopwatch.Stop();

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    for (int i = 0; i < triads.Count; i++)
                    {
                        DelaunayTriangulator.Triad triad = triads[i];
                        Vector2 v0 = new Vector2(vertices[triad.a].x, vertices[triad.a].y);
                        Vector2 v1 = new Vector2(vertices[triad.b].x, vertices[triad.b].y);
                        Vector2 v2 = new Vector2(vertices[triad.c].x, vertices[triad.c].y);
                        Raylib.DrawTriangleLines(v0, v1, v2, Color.GREEN);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"s-hull:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_MIConvexHull(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 4) return;

                // vertices
                List<vec2QHull> vertices = new List<vec2QHull>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices.Add(new vec2QHull(vec.X, vec.Y));
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                var triangles = MIConvexHull.Triangulation.CreateDelaunay<vec2QHull, triangleQHull>(vertices).Cells;
                stopwatch.Stop();

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    foreach (var triangle in triangles)
                    {
                        Vector2 v0 = new Vector2((float)triangle.Vertices[0].x, (float)triangle.Vertices[0].y);
                        Vector2 v1 = new Vector2((float)triangle.Vertices[1].x, (float)triangle.Vertices[1].y);
                        Vector2 v2 = new Vector2((float)triangle.Vertices[2].x, (float)triangle.Vertices[2].y);
                        Raylib.DrawTriangleLines(v0, v1, v2, Color.BLACK);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"MIConvexHull:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_Poly2Tri(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                var vertices = new List<Poly2Tri.Vector2>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices.Add(new Poly2Tri.Vector2(vec.X, vec.Y));
                }

                // calculate
                var triangles = new List<Poly2Tri.Triangle>();
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                Poly2Tri.Shape shape = new Poly2Tri.Shape(vertices);
                shape.Triangulate(triangles);
                stopwatch.Stop();

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    foreach (var triangle in triangles)
                    {
                        Vector2 v0 = new Vector2((float)triangle.Points[0].X, (float)triangle.Points[0].Y);
                        Vector2 v1 = new Vector2((float)triangle.Points[1].X, (float)triangle.Points[1].Y);
                        Vector2 v2 = new Vector2((float)triangle.Points[2].X, (float)triangle.Points[2].Y);
                        Raylib.DrawTriangleLines(v0, v1, v2, Color.YELLOW);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"Poly2Tri:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_Poly2Trics(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                var vertices = new List<Poly2Tri_cs.PolygonPoint>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices.Add(new Poly2Tri_cs.PolygonPoint(vec.X, vec.Y));
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                Poly2Tri_cs.Polygon polygon = new Poly2Tri_cs.Polygon(vertices);
                Poly2Tri_cs.P2T.Triangulate(polygon);
                stopwatch.Stop();
                var triangles = polygon.Triangles;

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    foreach (var triangle in triangles)
                    {
                        Vector2 v0 = new Vector2((float)triangle.Points[0].X, (float)triangle.Points[0].Y);
                        Vector2 v1 = new Vector2((float)triangle.Points[1].X, (float)triangle.Points[1].Y);
                        Vector2 v2 = new Vector2((float)triangle.Points[2].X, (float)triangle.Points[2].Y);
                        Raylib.DrawTriangleLines(v0, v1, v2, Color.PINK);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"poly2tri-cs:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_Poly2TriBiofluidix(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                var vertices = new List<Poly2Tri_biofluidix.PolygonPoint>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices.Add(new Poly2Tri_biofluidix.PolygonPoint(vec.X, vec.Y));
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                Poly2Tri_biofluidix.Polygon polygon = new Poly2Tri_biofluidix.Polygon(vertices);
                Poly2Tri_biofluidix.Poly2Tri.triangulate(polygon);
                stopwatch.Stop();
                var triangles = polygon.getTriangles();

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    foreach (var triangle in triangles)
                    {
                        Vector2 v0 = new Vector2((float)triangle.points[0].getX(), (float)triangle.points[0].getY());
                        Vector2 v1 = new Vector2((float)triangle.points[1].getX(), (float)triangle.points[1].getY());
                        Vector2 v2 = new Vector2((float)triangle.points[2].getX(), (float)triangle.points[2].getY());
                        Raylib.DrawTriangleLines(v0, v1, v2, Color.BROWN);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"Poly2Tri-biofluidix:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_LibTessDotNet(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                List<LibTessDotNet.ContourVertex> vertices = new List<LibTessDotNet.ContourVertex>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices.Add(new LibTessDotNet.ContourVertex(new LibTessDotNet.Vec3(vec.X, vec.Y, 0)));
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                LibTessDotNet.Tess tess = new LibTessDotNet.Tess();
                tess.AddContour(vertices, LibTessDotNet.ContourOrientation.Original);
                tess.Tessellate(LibTessDotNet.WindingRule.Positive, LibTessDotNet.ElementType.Polygons);
                stopwatch.Stop();
                var libtessdotnetVertices = tess.Vertices;
                var libtessdotnetTriangles = tess.Elements;

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    for (int i = 0; i < libtessdotnetTriangles.Length; i += 3)
                    {
                        LibTessDotNet.Vec3 v0 = libtessdotnetVertices[libtessdotnetTriangles[i + 0]].Position;
                        LibTessDotNet.Vec3 v1 = libtessdotnetVertices[libtessdotnetTriangles[i + 1]].Position;
                        LibTessDotNet.Vec3 v2 = libtessdotnetVertices[libtessdotnetTriangles[i + 2]].Position;
                        Raylib.DrawTriangleLines(new Vector2(v0.X, v0.Y), new Vector2(v1.X, v1.Y), new Vector2(v2.X, v2.Y), Color.RAYWHITE);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"LibTessDotNet:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_DelaunayOskar(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 3) return;

                // vertices
                List<Vector2> vertices = new List<Vector2>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices.Add(new Vector2(vec.X, vec.Y));
                }

                // calculate
                GK.DelaunayCalculator delaunayCalculator = new GK.DelaunayCalculator();
                var delaunay_oskar = new GK.DelaunayTriangulation();
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                delaunayCalculator.CalculateTriangulation(vertices, ref delaunay_oskar);
                stopwatch.Stop();

                // draw
                Raylib.BeginMode2D(camera2D);
                {
                    for (int i = 0; i < delaunay_oskar.Triangles.Count; i += 3)
                    {
                        Vector2 v0 = delaunay_oskar.Vertices[delaunay_oskar.Triangles[i + 0]];
                        Vector2 v1 = delaunay_oskar.Vertices[delaunay_oskar.Triangles[i + 1]];
                        Vector2 v2 = delaunay_oskar.Vertices[delaunay_oskar.Triangles[i + 2]];
                        Raylib.DrawTriangleLines(v0, v1, v2, Color.SKYBLUE);
                    }
                }
                Raylib.EndMode2D();
                Raylib.DrawText($"Delaunay-Oskar:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_QuickHull3DBiofluidix(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 4) return;

                // vertices
                QuickHull3D.Point3d[] vertices = new QuickHull3D.Point3d[inPoints.Count];
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices[i] = new QuickHull3D.Point3d(vec.X, vec.Y, Raylib.GetRandomValue(-1000, 1000));
                }

                // calculate
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                QuickHull3D.Hull hull = new QuickHull3D.Hull(vertices);
                hull.Triangulate();
                stopwatch.Stop();

                // draw
                Raylib.DrawText($"QuickHull3D-biofluidix:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }

            void test_QuickHull3DOskar(Camera2D camera2D, int fontPositionX, int fontPositionY, int fontSize, List<Vector2> inPoints, Color color)
            {
                if (inPoints.Count < 4) return;

                // vertices
                List<Vector3> vertices = new List<Vector3>();
                for (int i = 0; i < inPoints.Count; i++)
                {
                    Vector2 vec = inPoints[i];
                    if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                        continue;
                    vertices.Add(new Vector3(vec.X, vec.Y, Raylib.GetRandomValue(-1000, 1000)));
                }

                // calculate
                List<Vector3> outVertices = new List<Vector3>();
                List<Vector3> outNormals = new List<Vector3>();
                List<int> outTriangles = new List<int>();
                GK.ConvexHullCalculator hull = new GK.ConvexHullCalculator();
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                hull.GenerateHull(vertices, false, ref outVertices, ref outTriangles, ref outNormals);
                stopwatch.Stop();

                // draw
                Raylib.DrawText($"QuickHull3D-oskar:{stopwatch.Elapsed.TotalMilliseconds}", fontPositionX, fontPositionY, fontSize, color);
            }
        }
    }
}