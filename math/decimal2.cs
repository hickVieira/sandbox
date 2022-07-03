using System;
using System.Numerics;

namespace Sandbox
{
    public struct decimal2
    {
        public decimal x;
        public decimal y;

        public decimal2(decimal x, decimal y)
        {
            this.x = x;
            this.y = y;
        }

        public decimal2(decimal x)
        {
            this.x = x;
            this.y = x;
        }

        public static implicit operator decimal2(Vector2 a) => new decimal2((decimal)a.X, (decimal)a.Y);
        public static explicit operator Vector2(decimal2 a) => new Vector2((float)a.x, (float)a.y);

        public static decimal2 operator +(decimal2 a, decimal2 b) => new decimal2(a.x + b.x, a.y + b.y);
        public static decimal2 operator -(decimal2 a, decimal2 b) => new decimal2(a.x - b.x, a.y - b.y);

        public static decimal2 operator *(decimal2 a, float b) => new decimal2(a.x * (decimal)b, a.y * (decimal)b);
        public static decimal2 operator *(decimal2 a, double b) => new decimal2(a.x * (decimal)b, a.y * (decimal)b);
        public static decimal2 operator *(decimal2 a, decimal b) => new decimal2(a.x * (decimal)b, a.y * (decimal)b);
        public static decimal2 operator *(float a, decimal2 b) => new decimal2((decimal)a * b.x, (decimal)a * b.y);
        public static decimal2 operator *(double a, decimal2 b) => new decimal2((decimal)a * b.x, (decimal)a * b.y);
        public static decimal2 operator *(decimal a, decimal2 b) => new decimal2((decimal)a * b.x, (decimal)a * b.y);

        public static decimal2 operator /(decimal2 a, float b) => new decimal2(a.x / (decimal)b, a.y / (decimal)b);
        public static decimal2 operator /(decimal2 a, double b) => new decimal2(a.x / (decimal)b, a.y / (decimal)b);
        public static decimal2 operator /(decimal2 a, decimal b) => new decimal2(a.x / (decimal)b, a.y / (decimal)b);
        public static decimal2 operator /(float a, decimal2 b) => new decimal2((decimal)a / b.x, (decimal)a / b.y);
        public static decimal2 operator /(double a, decimal2 b) => new decimal2((decimal)a / b.x, (decimal)a / b.y);
        public static decimal2 operator /(decimal a, decimal2 b) => new decimal2((decimal)a / b.x, (decimal)a / b.y);
    }

    public static partial class mathh
    {
        public static decimal distance(decimal2 a, decimal2 b) { decimal2 dif = b - a; return mathh.sqrt(dif.x * dif.x + dif.y * dif.y); }
        public static decimal lengthsq(decimal2 a) { return a.x * a.x + a.y * a.y; }
        public static decimal length(decimal2 a) { return mathh.sqrt(lengthsq(a)); }
        public static decimal2 perpendicular(decimal2 a) { return new decimal2(a.y, -a.x); }
    }
}