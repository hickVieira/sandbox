using System.Collections.Generic;

namespace Sandbox
{
    public static class Delaunay
    {
        [System.Serializable]
        public struct Triangle
        {
            public struct Edge : System.IEquatable<Edge>
            {
                public int _vIndex0;
                public int _vIndex1;

                public Edge(int vIndex0, int vIndex1)
                {
                    this._vIndex0 = vIndex0;
                    this._vIndex1 = vIndex1;
                }

                public bool Equals(Edge other)
                {
                    return (this._vIndex0 == other._vIndex0 && this._vIndex1 == other._vIndex1) || (this._vIndex0 == other._vIndex1 && this._vIndex1 == other._vIndex0);
                }
            }

            public int _vIndex0;
            public int _vIndex1;
            public int _vIndex2;
            public Edge _edge0;
            public Edge _edge1;
            public Edge _edge2;
            public decimal2 _circumCenter;
            public decimal _circumRadius;

            public Triangle(int vIndex0, int vIndex1, int vIndex2, decimal2 v0, decimal2 v1, decimal2 v2)
            {
                this._vIndex0 = vIndex0;
                this._vIndex1 = vIndex1;
                this._vIndex2 = vIndex2;
                this._edge0 = new Edge(vIndex0, vIndex1);
                this._edge1 = new Edge(vIndex1, vIndex2);
                this._edge2 = new Edge(vIndex2, vIndex0);

                decimal2 edge0 = v1 - v0;
                decimal2 edge1 = v2 - v1;
                decimal2 edge2 = v0 - v2;

                decimal2 mid0 = (v0 + v1) / 2m;
                decimal2 mid1 = (v1 + v2) / 2m;
                decimal2 bisector0 = mathh.perpendicular(edge0);
                decimal2 bisector1 = mathh.perpendicular(edge1);

                this._circumCenter = RayRayIntersection(mid0, bisector0, mid1, bisector1);
                this._circumRadius = mathh.distance(_circumCenter, v0);
            }

            public bool IsInsideCircumcircle(decimal2 point)
            {
                return mathh.distance(_circumCenter, point) < _circumRadius + EPSILON_RADIUS;
            }
        }

        public static readonly decimal EPSILON_RADIUS = (decimal)0.00001m;
        public static readonly decimal EPSILON_COPLANAR = (decimal)0.0001m;

        // https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
        public static List<Triangle> BowyerWatson2D(List<decimal2> inVertices, decimal superTriangleSize)
        {
            List<decimal2> inVerticesTemp = new List<decimal2>(inVertices);
            List<Triangle> outTriangles = new List<Triangle>();

            // add super triangle
            int superTriangleVIndex = inVerticesTemp.Count;
            decimal2 v0 = new decimal2(0, superTriangleSize);
            decimal2 v1 = new decimal2(superTriangleSize, -superTriangleSize);
            decimal2 v2 = new decimal2(-superTriangleSize, -superTriangleSize);

            inVerticesTemp.Add(v0);
            inVerticesTemp.Add(v1);
            inVerticesTemp.Add(v2);

            outTriangles.Add(new Triangle(superTriangleVIndex + 0, superTriangleVIndex + 1, superTriangleVIndex + 2, v0, v1, v2));

            for (int i = 0; i < inVerticesTemp.Count - 3; i++)
            {
                decimal2 currentVertex = inVerticesTemp[i];
                List<int> badTriangles = new List<int>();
                List<Triangle.Edge> polygon = new List<Triangle.Edge>();

                // make list of triangles colliding with current point
                for (int j = 0; j < outTriangles.Count; j++)
                {
                    Triangle currentTriangle = outTriangles[j];
                    if (currentTriangle.IsInsideCircumcircle(currentVertex))
                        badTriangles.Add(j);
                }

                // find triangles edges
                for (int j = 0; j < badTriangles.Count; j++)
                {
                    Triangle currentTriangle = outTriangles[badTriangles[j]];

                    // check edges
                    if (!HasSharingEdge(outTriangles, badTriangles, currentTriangle._edge0, j))
                        polygon.Add(currentTriangle._edge0);
                    if (!HasSharingEdge(outTriangles, badTriangles, currentTriangle._edge1, j))
                        polygon.Add(currentTriangle._edge1);
                    if (!HasSharingEdge(outTriangles, badTriangles, currentTriangle._edge2, j))
                        polygon.Add(currentTriangle._edge2);
                }

                // sort to remove last ones first, so we dont lose indices
                badTriangles.Sort();
                for (int j = badTriangles.Count - 1; j > -1; j--)
                    outTriangles.RemoveAt(badTriangles[j]);

                // build new triangles
                for (int j = 0; j < polygon.Count; j++)
                {
                    Triangle.Edge currentEdge = polygon[j];

                    decimal2 edgeVertex0 = inVerticesTemp[currentEdge._vIndex0];
                    decimal2 edgeVertex1 = inVerticesTemp[currentEdge._vIndex1];
                    Triangle newTriangle = new Triangle(currentEdge._vIndex0, currentEdge._vIndex1, i, edgeVertex0, edgeVertex1, currentVertex);
                    outTriangles.Add(newTriangle);
                }
            }

            // remove super triangles
            // same strategey as aboeve, save indices to list then sortand remove lasts first
            List<int> superTriangleIndices = new List<int>();
            for (int i = 0; i < outTriangles.Count; i++)
            {
                Triangle currentTriangle = outTriangles[i];

                bool isSuperTriangle0 = (currentTriangle._vIndex0 == superTriangleVIndex + 0) || (currentTriangle._vIndex0 == superTriangleVIndex + 1) || (currentTriangle._vIndex0 == superTriangleVIndex + 2);
                bool isSuperTriangle1 = (currentTriangle._vIndex1 == superTriangleVIndex + 0) || (currentTriangle._vIndex1 == superTriangleVIndex + 1) || (currentTriangle._vIndex1 == superTriangleVIndex + 2);
                bool isSuperTriangle2 = (currentTriangle._vIndex2 == superTriangleVIndex + 0) || (currentTriangle._vIndex2 == superTriangleVIndex + 1) || (currentTriangle._vIndex2 == superTriangleVIndex + 2);

                if (isSuperTriangle0 || isSuperTriangle1 || isSuperTriangle2)
                    superTriangleIndices.Add(i);
            }

            superTriangleIndices.Sort();
            for (int i = superTriangleIndices.Count - 1; i > -1; i--)
                outTriangles.RemoveAt(superTriangleIndices[i]);

            return outTriangles;
        }

        static bool HasSharingEdge(List<Triangle> triangles, List<int> badTriangles, Triangle.Edge edge, int ignoreIndex)
        {
            for (int i = 0; i < badTriangles.Count; i++)
            {
                if (i == ignoreIndex) continue;

                Triangle otherTriangle = triangles[badTriangles[i]];

                if (edge.Equals(otherTriangle._edge0) || edge.Equals(otherTriangle._edge1) || edge.Equals(otherTriangle._edge2))
                    return true;
            }

            return false;
        }

        // https://mathworld.wolfram.com/Circumradius.html
        // no need for triangle circum radius - just calculate circum center, then distance to any vert is radius
        static decimal TriangleCircumradius(decimal2 v0, decimal2 v1, decimal2 v2, decimal2 edge0, decimal2 edge1, decimal2 edge2)
        {
            decimal a = mathh.length(edge0);
            decimal b = mathh.length(edge1);
            decimal c = mathh.length(edge2);
            decimal perimeter = a + b + c;
            decimal s = perimeter / 2;

            decimal r = (a * b * c) / (4 * mathh.sqrt(s * (a + b - s) * (a + c - s) * (b + c - s)));
            return r;
        }

        // https://www.geeksforgeeks.org/program-find-line-passing-2-points/
        static void LineFromPoints(decimal2 p0, decimal2 p1, out decimal a, out decimal b, out decimal c)
        {
            a = p1.y - p0.y;
            b = p0.x - p1.x;
            c = a * p0.x + b * p0.y;
        }

        // https://stackoverflow.com/questions/8694323/how-to-detect-if-the-line-intersects-in-c
        static decimal2 RayRayIntersection(decimal2 rayOrigin0, decimal2 rayDirection0, decimal2 rayOrigin1, decimal2 rayDirection1)
        {
            decimal a0, b0, c0;
            LineFromPoints(rayOrigin0, rayOrigin0 + rayDirection0, out a0, out b0, out c0);

            decimal a1, b1, c1;
            LineFromPoints(rayOrigin1, rayOrigin1 + rayDirection1, out a1, out b1, out c1);

            decimal delta = a0 * b1 - a1 * b0;
            if (delta == 0)
                return new decimal2(0);

            decimal x = (b1 * c0 - b0 * c1) / delta;
            decimal y = (a0 * c1 - a1 * c0) / delta;

            return new decimal2(x, y);
        }
    }
}