using System;
using System.Numerics;
using System.Collections.Generic;
using Raylib_cs;

namespace Sandbox
{
    public struct AABB
    {
        public Vector2 _center;
        public Vector2 _extents;

        public Vector2 min { get => _center - _extents; }
        public Vector2 max { get => _center + _extents; }
        public float width { get => _extents.X * 2; }
        public float height { get => _extents.Y * 2; }

        public AABB(Vector2 center, Vector2 size)
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

        public bool Contains(AABB aabb)
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

    [System.Serializable]
    public class QuadTree<T>
    {
        [System.Serializable]
        public struct Node
        {
            public int _childrenIndex = -1;
            public byte _depth = 0;

            public Node(int childrenIndex, byte depth)
            {
                this._childrenIndex = childrenIndex;
                this._depth = depth;
            }
        }
        private List<Node> _nodes;
        public List<List<T>> _nodesItems;
        public List<AABB> _nodesAABBs;
        private Vector2 _maxSize;
        private uint _maxDepth;
        private uint[] _insertBuffer;
        private uint _insertBufferCount;

        public void Draw(Color color)
        {
            if (_nodesAABBs == null)
                return;
            for (int i = 0; i < _nodesAABBs.Count; i++)
                _nodesAABBs[i].Draw(color);
        }

        public QuadTree(Vector2 center, Vector2 size, byte depth)
        {
            _maxSize = size;
            _maxDepth = depth;
            _nodes = new List<Node>();
            _nodesItems = new List<List<T>>();
            _nodesAABBs = new List<AABB>();
            int subdivides = depth + 1;
            _insertBuffer = new uint[subdivides * subdivides];
            CreateNode(center, size, depth);
        }

        public void Insert(T obj, Vector2 position)
        {
            Insert(obj, position, 0);
        }

        void Insert(T obj, Vector2 position, int nodeIndex)
        {
            AABB currentNodeAABB = _nodesAABBs[nodeIndex];
            if (currentNodeAABB.Contains(position))
            {
                if (_nodes[nodeIndex]._depth == 0)
                {
                    _nodesItems[nodeIndex] ??= new List<T>();
                    _nodesItems[nodeIndex].Add(obj);
                    return;
                }

                if (_nodes[nodeIndex]._childrenIndex == -1)
                    SplitNode(nodeIndex);

                Insert(obj, position, _nodes[nodeIndex]._childrenIndex + 0);
                Insert(obj, position, _nodes[nodeIndex]._childrenIndex + 1);
                Insert(obj, position, _nodes[nodeIndex]._childrenIndex + 2);
                Insert(obj, position, _nodes[nodeIndex]._childrenIndex + 3);
            }
        }

        public void Insert(T obj, AABB aabb)
        {
            Insert(obj, aabb, 0);
        }

        void Insert(T obj, AABB aabb, int nodeIndex)
        {
            AABB currentNodeAABB = _nodesAABBs[nodeIndex];
            if (currentNodeAABB.Contains(aabb))
            {
                if (_nodes[nodeIndex]._depth == 0)
                {
                    _nodesItems[nodeIndex] ??= new List<T>();
                    _nodesItems[nodeIndex].Add(obj);
                    return;
                }

                if (_nodes[nodeIndex]._childrenIndex == -1)
                    SplitNode(nodeIndex);

                Insert(obj, aabb, _nodes[nodeIndex]._childrenIndex + 0);
                Insert(obj, aabb, _nodes[nodeIndex]._childrenIndex + 1);
                Insert(obj, aabb, _nodes[nodeIndex]._childrenIndex + 2);
                Insert(obj, aabb, _nodes[nodeIndex]._childrenIndex + 3);
            }
        }

        Node CreateNode(Vector2 center, Vector2 size, byte depth)
        {
            Node node = new Node(-1, depth);
            _nodes.Add(node);
            _nodesItems.Add(null);
            _nodesAABBs.Add(new AABB(center, size));
            return node;
        }

        Node CreateNodeRecursive(int nodeIndex, Vector2 center, Vector2 size, int depth)
        {
            Node node = CreateNode(center, size, (byte)depth);
            if (depth > -1)
            {
                size /= 2;
                depth--;
                _nodes[nodeIndex] = new Node(_nodes.Count, (byte)depth);
                CreateNodeRecursive(_nodes.Count, center + new Vector2(0.5f, 0.5f) * size, size, depth);
                CreateNodeRecursive(_nodes.Count, center + new Vector2(-0.5f, 0.5f) * size, size, depth);
                CreateNodeRecursive(_nodes.Count, center + new Vector2(-0.5f, -0.5f) * size, size, depth);
                CreateNodeRecursive(_nodes.Count, center + new Vector2(0.5f, -0.5f) * size, size, depth);
            }
            return node;
        }

        void SplitNode(int nodeIndex)
        {
            Node node = _nodes[nodeIndex];
            AABB aabb = _nodesAABBs[nodeIndex];
            _nodes[nodeIndex] = new Node(_nodes.Count, node._depth);

            Vector2 center = aabb._center;
            Vector2 size = aabb._extents; // * 2 / 2 -> we need to divide size by 2, but extents is exactly that
            byte depth = (byte)(node._depth - 1);
            CreateNode(center + new Vector2(0.5f, 0.5f) * size, size, depth);
            CreateNode(center + new Vector2(-0.5f, 0.5f) * size, size, depth);
            CreateNode(center + new Vector2(-0.5f, -0.5f) * size, size, depth);
            CreateNode(center + new Vector2(0.5f, -0.5f) * size, size, depth);
        }

        public List<T> GetNodeItems(int nodeIndex)
        {
            return _nodesItems[nodeIndex];
        }

        public AABB GetNodeAABB(int nodeIndex)
        {
            return _nodesAABBs[nodeIndex];
        }

        public int GetNodeIndex(Vector2 position, uint depth)
        {
            if (!_nodesAABBs[0].Contains(position))
                return -1;

            int nodeIndex = GetNodeIndex(0, position, Math.Min(_maxDepth, depth));
            if (nodeIndex < 0 || nodeIndex >= _nodes.Count)
                return -1;

            return nodeIndex;
        }

        int GetNodeIndex(int nodeIndex, Vector2 position, uint depth)
        {
            if (!_nodesAABBs[nodeIndex].Contains(position))
                return -1;

            Node node = _nodes[nodeIndex];

            if (node._depth == depth)
                return nodeIndex;

            if (node._childrenIndex != -1)
                for (int i = 0; i < 4; i++)
                {
                    int leafNode = GetNodeIndex(node._childrenIndex + i, position, depth);
                    if (leafNode != -1)
                        return leafNode;
                }

            return -1;
        }
    }
}
