using System.Numerics;
using Raylib_cs;

namespace Sandbox
{
    public struct AABB2
    {
        public Vector2 _center;
        public Vector2 _extents;

        public Vector2 min { get => _center - _extents; }
        public Vector2 max { get => _center + _extents; }
        public float width { get => _extents.X * 2; }
        public float height { get => _extents.Y * 2; }

        public AABB2(Vector2 center, Vector2 size)
        {
            this._center = center;
            this._extents = size / 2;
        }

        public bool Contains(Vector2 position)
        {
            bool containsX = (position.X > _center.X - _extents.X) && (position.X < _center.X + _extents.X);
            bool containsY = (position.Y > _center.Y - _extents.Y) && (position.Y < _center.Y + _extents.Y);
            return containsX && containsY;
        }

        public bool Contains(AABB2 aabb)
        {
            Vector2 thisMin = min;
            Vector2 otherMin = aabb.min;

            return (thisMin.X < otherMin.X + aabb.width &&
                    thisMin.X + width > otherMin.X &&
                    thisMin.Y < otherMin.Y + aabb.height &&
                    thisMin.Y + height > otherMin.Y);
        }

        public void Draw(Color color)
        {
            Vector2 position = min;
            Raylib.DrawRectangleLines((int)position.X, (int)position.Y, (int)width, (int)height, color);
        }
    }
}
