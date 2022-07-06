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
            int _screenWidth = (int)(800 * 1.5f);
            int _screenHeight = (int)(600 * 1.5f);
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
                    camera2D.zoom = Raylib_cs.Raymath.Clamp(camera2D.zoom, 0.025f, 3);
                    if (Raylib.IsMouseButtonDown(MouseButton.MOUSE_MIDDLE_BUTTON))
                        camera2D.target -= mouseDelta / camera2D.zoom;
                    Vector2 mouseScreenPos = Raylib.GetMousePosition();
                    Vector2 mouseWorldPos = Raylib.GetScreenToWorld2D(mouseScreenPos, camera2D);

                    bool isClick = Raylib.IsMouseButtonPressed(MouseButton.MOUSE_BUTTON_LEFT);
                    bool isHold = Raylib.IsMouseButtonDown(MouseButton.MOUSE_BUTTON_RIGHT);

                    // create vertex
                    if (isClick)
                        points.Add(mouseWorldPos);
                    if (isHold && points.Count > 0 && Vector2.Distance(mouseWorldPos, points[points.Count - 1]) > 4)
                        points.Add(mouseWorldPos);

                    if (Raylib.IsKeyPressed(KeyboardKey.KEY_R))
                        points.Clear();

                    System.Diagnostics.Stopwatch stopwatchMydelaunay = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchNet = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchSharp = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchCsharp = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchSHull = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchQHull = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchPoly2Tri = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchPoly2Trics = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchPoly2Tri_biofluidix = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchLibTessDotNet = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchQuickHull3D_biofluidix = new System.Diagnostics.Stopwatch();
                    System.Diagnostics.Stopwatch stopwatchQuickHull3D_oskar = new System.Diagnostics.Stopwatch();

                    List<decimal2> mydelaunayVertices = null;
                    List<Delaunay.Triangle> mydelaunayTriangles = null;
                    DelaunatorNet.TriangulationInfo netTriangleInfo = null;
                    int[] sharpTriangles = null;
                    DelaunatorSharp.IPoint[] sharpPoints = null;
                    List<int> csharpTriangles = null;
                    List<double> csharpVertices = null;
                    List<DelaunayTriangulator.Triad> shullTriads = null;
                    List<DelaunayTriangulator.Vertex> shullPoints = null;
                    IEnumerable<triangleQHull> qhullTriangles = null;
                    List<Poly2Tri.Triangle> poly2triTriangles = null;
                    List<Poly2Tri.Vector2> poly2triVertices = null;
                    IList<Poly2Tri_cs.DelaunayTriangle> poly2tricsTriangles = null;
                    List<Poly2Tri_cs.PolygonPoint> poly2tricsVertices = null;
                    List<Poly2Tri_biofluidix.PolygonPoint> poly2tribiofluidixVertices = null;
                    IList<Poly2Tri_biofluidix.DelaunayTriangle> poly2tribiofluidixTriangles = null;
                    int[] libtessdotnetTriangles = null;
                    LibTessDotNet.ContourVertex[] libtessdotnetVertices = null;
                    if (points.Count > 2)
                    {
                        {
                            mydelaunayVertices = new List<decimal2>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                mydelaunayVertices.Add(vec);
                            }
                            stopwatchMydelaunay.Start();
                            mydelaunayTriangles = Delaunay.BowyerWatson2D(mydelaunayVertices, 10000);
                            stopwatchMydelaunay.Stop();
                        }
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
                            stopwatchNet.Start();
                            DelaunatorNet.Delaunator delaunator = new DelaunatorNet.Delaunator(vertices);
                            netTriangleInfo = delaunator.Build();
                            stopwatchNet.Stop();
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
                            stopwatchSharp.Start();
                            DelaunatorSharp.Delaunator delaunator = new DelaunatorSharp.Delaunator(vertices);
                            stopwatchSharp.Stop();
                            sharpTriangles = delaunator.Triangles;
                            sharpPoints = delaunator.Points;
                        }
                        {
                            csharpVertices = new List<double>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                csharpVertices.Add(vec.X);
                                csharpVertices.Add(vec.Y);
                            }
                            stopwatchCsharp.Start();
                            Delaunator.Triangulation triangulation = new Delaunator.Triangulation(csharpVertices);
                            stopwatchCsharp.Stop();
                            csharpTriangles = triangulation.triangles;
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
                            stopwatchSHull.Start();
                            DelaunayTriangulator.Triangulator triangulator = new DelaunayTriangulator.Triangulator();
                            shullTriads = triangulator.Triangulation(shullPoints, true);
                            stopwatchSHull.Stop();
                        }
                        if (points.Count > 3) // this should work with 2 verts - probaly a bug
                        {
                            List<vec2QHull> vertices = new List<vec2QHull>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                vertices.Add(new vec2QHull(vec.X, vec.Y));
                            }
                            stopwatchQHull.Start();
                            qhullTriangles = MIConvexHull.Triangulation.CreateDelaunay<vec2QHull, triangleQHull>(vertices).Cells;
                            stopwatchQHull.Stop();
                        }
                        {
                            poly2triVertices = new List<Poly2Tri.Vector2>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                poly2triVertices.Add(new Poly2Tri.Vector2(vec.X, vec.Y));
                            }
                            poly2triTriangles ??= new List<Poly2Tri.Triangle>();
                            stopwatchPoly2Tri.Start();
                            Poly2Tri.Shape shape = new Poly2Tri.Shape(poly2triVertices);
                            shape.Triangulate(poly2triTriangles);
                            stopwatchPoly2Tri.Stop();
                        }
                        {
                            poly2tricsVertices = new List<Poly2Tri_cs.PolygonPoint>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                poly2tricsVertices.Add(new Poly2Tri_cs.PolygonPoint(vec.X, vec.Y));
                            }
                            stopwatchPoly2Trics.Start();
                            Poly2Tri_cs.Polygon polygon = new Poly2Tri_cs.Polygon(poly2tricsVertices);
                            Poly2Tri_cs.P2T.Triangulate(polygon);
                            stopwatchPoly2Trics.Stop();
                            poly2tricsTriangles = polygon.Triangles;
                        }
                        {
                            poly2tribiofluidixVertices = new List<Poly2Tri_biofluidix.PolygonPoint>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                poly2tribiofluidixVertices.Add(new Poly2Tri_biofluidix.PolygonPoint(vec.X, vec.Y));
                            }
                            stopwatchPoly2Tri_biofluidix.Start();
                            Poly2Tri_biofluidix.Polygon polygon = new Poly2Tri_biofluidix.Polygon(poly2tribiofluidixVertices);
                            Poly2Tri_biofluidix.Poly2Tri.triangulate(polygon);
                            stopwatchPoly2Tri_biofluidix.Stop();
                            poly2tribiofluidixTriangles = polygon.getTriangles();
                        }
                        {
                            List<LibTessDotNet.ContourVertex> vertices = new List<LibTessDotNet.ContourVertex>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                vertices.Add(new LibTessDotNet.ContourVertex(new LibTessDotNet.Vec3(vec.X, vec.Y, 0)));
                            }
                            stopwatchLibTessDotNet.Start();
                            LibTessDotNet.Tess tess = new LibTessDotNet.Tess();
                            tess.AddContour(vertices, LibTessDotNet.ContourOrientation.Original);
                            tess.Tessellate(LibTessDotNet.WindingRule.Positive, LibTessDotNet.ElementType.Polygons);
                            stopwatchLibTessDotNet.Stop();
                            libtessdotnetVertices = tess.Vertices;
                            libtessdotnetTriangles = tess.Elements;
                        }
                        if (points.Count > 3)
                        {
                            List<QuickHull3D.Point3d> vertices = new List<QuickHull3D.Point3d>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                vertices.Add(new QuickHull3D.Point3d(vec.X, vec.Y, Raylib.GetRandomValue(-1000, 1000)));
                            }
                            stopwatchQuickHull3D_biofluidix.Start();
                            QuickHull3D.Hull hull = new QuickHull3D.Hull(vertices.ToArray());
                            hull.Triangulate();
                            stopwatchQuickHull3D_biofluidix.Stop();
                        }
                        if (points.Count > 3)
                        {
                            List<Vector3> vertices = new List<Vector3>();
                            for (int i = 0; i < points.Count; i++)
                            {
                                Vector2 vec = points[i];
                                if (float.IsNaN(vec.X) || float.IsNaN(vec.Y))
                                    continue;
                                vertices.Add(new Vector3(vec.X, vec.Y, Raylib.GetRandomValue(-1000, 1000)));
                            }
                            List<Vector3> outVertices = new List<Vector3>();
                            List<Vector3> outNormals = new List<Vector3>();
                            List<int> outTriangles = new List<int>();
                            stopwatchQuickHull3D_oskar.Start();
                            GK.ConvexHullCalculator hull = new GK.ConvexHullCalculator();
                            hull.GenerateHull(vertices, false, ref outVertices, ref outTriangles, ref outNormals);
                            stopwatchQuickHull3D_oskar.Stop();
                        }
                    }

                    // render
                    Raylib.BeginDrawing();
                    {
                        Raylib.ClearBackground(Color.GRAY);
                        Raylib.BeginMode2D(camera2D);
                        {
                            for (int i = 0; i < points.Count; i++)
                                Raylib.DrawCircle((int)points[i].X, (int)points[i].Y, 1, Color.BLACK);

                            if (mydelaunayTriangles != null)
                            {
                                for (int i = 0; i < mydelaunayTriangles.Count; i++)
                                {
                                    int vIndex0 = mydelaunayTriangles[i]._vIndex0;
                                    int vIndex1 = mydelaunayTriangles[i]._vIndex1;
                                    int vIndex2 = mydelaunayTriangles[i]._vIndex2;
                                    Raylib.DrawTriangleLines((Vector2)mydelaunayVertices[vIndex0], (Vector2)mydelaunayVertices[vIndex1], (Vector2)mydelaunayVertices[vIndex2], Color.BEIGE);
                                }
                            }
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
                            if (csharpTriangles != null)
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
                                foreach (var triangle in qhullTriangles)
                                {
                                    Vector2 v0 = new Vector2((float)triangle.Vertices[0].x, (float)triangle.Vertices[0].y);
                                    Vector2 v1 = new Vector2((float)triangle.Vertices[1].x, (float)triangle.Vertices[1].y);
                                    Vector2 v2 = new Vector2((float)triangle.Vertices[2].x, (float)triangle.Vertices[2].y);
                                    Raylib.DrawTriangleLines(v0, v1, v2, Color.BLACK);
                                }
                            }
                            if (poly2triTriangles != null)
                            {
                                foreach (var triangle in poly2triTriangles)
                                {
                                    Vector2 v0 = new Vector2((float)triangle.Points[0].X, (float)triangle.Points[0].Y);
                                    Vector2 v1 = new Vector2((float)triangle.Points[1].X, (float)triangle.Points[1].Y);
                                    Vector2 v2 = new Vector2((float)triangle.Points[2].X, (float)triangle.Points[2].Y);
                                    Raylib.DrawTriangleLines(v0, v1, v2, Color.YELLOW);
                                }
                            }
                            if (poly2tricsTriangles != null)
                            {
                                foreach (var triangle in poly2tricsTriangles)
                                {
                                    Vector2 v0 = new Vector2((float)triangle.Points[0].X, (float)triangle.Points[0].Y);
                                    Vector2 v1 = new Vector2((float)triangle.Points[1].X, (float)triangle.Points[1].Y);
                                    Vector2 v2 = new Vector2((float)triangle.Points[2].X, (float)triangle.Points[2].Y);
                                    Raylib.DrawTriangleLines(v0, v1, v2, Color.PINK);
                                }
                            }
                            if (poly2tribiofluidixTriangles != null)
                            {
                                foreach (var triangle in poly2tribiofluidixTriangles)
                                {
                                    Vector2 v0 = new Vector2((float)triangle.points[0].getX(), (float)triangle.points[0].getY());
                                    Vector2 v1 = new Vector2((float)triangle.points[1].getX(), (float)triangle.points[1].getY());
                                    Vector2 v2 = new Vector2((float)triangle.points[2].getX(), (float)triangle.points[2].getY());
                                    Raylib.DrawTriangleLines(v0, v1, v2, Color.BROWN);
                                }
                            }
                            if (libtessdotnetTriangles != null)
                            {
                                for (int i = 0; i < libtessdotnetTriangles.Length; i += 3)
                                {
                                    LibTessDotNet.Vec3 v0 = libtessdotnetVertices[libtessdotnetTriangles[i + 0]].Position;
                                    LibTessDotNet.Vec3 v1 = libtessdotnetVertices[libtessdotnetTriangles[i + 1]].Position;
                                    LibTessDotNet.Vec3 v2 = libtessdotnetVertices[libtessdotnetTriangles[i + 2]].Position;
                                    Raylib.DrawTriangleLines(new Vector2(v0.X, v0.Y), new Vector2(v1.X, v1.Y), new Vector2(v2.X, v2.Y), Color.RAYWHITE);
                                }
                            }
                            Raylib.DrawLine(short.MinValue, 0, short.MaxValue, 0, Color.RED);
                            Raylib.DrawLine(0, short.MinValue, 0, short.MaxValue, Color.GREEN);
                        }
                        Raylib.EndMode2D();

                        Raylib.DrawText($"myDelaunator:{stopwatchMydelaunay.Elapsed.TotalMilliseconds}", 12, 40 + 1, 20, Color.BEIGE);
                        Raylib.DrawText($"delaunator-net:{stopwatchNet.Elapsed.TotalMilliseconds}", 12, 60 + 1, 20, Color.RED);
                        Raylib.DrawText($"delaunator-sharp:{stopwatchSharp.Elapsed.TotalMilliseconds}", 12, 80 + 1, 20, Color.BLUE);
                        Raylib.DrawText($"delaunator-csharp:{stopwatchCsharp.Elapsed.TotalMilliseconds}", 12, 100 + 1, 20, Color.ORANGE);
                        Raylib.DrawText($"s-hull:{stopwatchSHull.Elapsed.TotalMilliseconds}", 12, 120 + 1, 20, Color.GREEN);
                        Raylib.DrawText($"MIConvexHull:{stopwatchQHull.Elapsed.TotalMilliseconds}", 12, 140 + 1, 20, Color.BLACK);
                        Raylib.DrawText($"Poly2Tri:{stopwatchPoly2Tri.Elapsed.TotalMilliseconds}", 12, 160 + 1, 20, Color.YELLOW);
                        Raylib.DrawText($"poly2tri-cs:{stopwatchPoly2Trics.Elapsed.TotalMilliseconds}", 12, 180 + 1, 20, Color.PINK);
                        Raylib.DrawText($"Poly2Tri-biofluidix:{stopwatchPoly2Tri_biofluidix.Elapsed.TotalMilliseconds}", 12, 200 + 1, 20, Color.BROWN);
                        Raylib.DrawText($"LibTessDotNet:{stopwatchLibTessDotNet.Elapsed.TotalMilliseconds}", 12, 220 + 1, 20, Color.RAYWHITE);
                        Raylib.DrawText($"QuickHull3D-biofluidix:{stopwatchQuickHull3D_biofluidix.Elapsed.TotalMilliseconds}", 12, 240 + 1, 20, Color.GOLD);
                        Raylib.DrawText($"QuickHull3D-oskar:{stopwatchQuickHull3D_oskar.Elapsed.TotalMilliseconds}", 12, 260 + 1, 20, Color.MAGENTA);
                    }
                    Raylib.EndDrawing();
                }
                Raylib.CloseWindow();
            }
        }
    }
}