using System;
using System.Numerics;
using System.Collections.Generic;
using Raylib_cs;

namespace Sandbox
{
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
        public List<AABB2> _nodesAABBs;
        private Vector2 _maxSize;
        private uint _maxDepth;

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
            _nodesAABBs = new List<AABB2>();
            CreateNode(center, size, depth);
        }

        public void Insert(T obj, Vector2 position)
        {
            Insert(obj, position, 0);
        }

        void Insert(T obj, Vector2 position, int nodeIndex)
        {
            AABB2 currentNodeAABB = _nodesAABBs[nodeIndex];
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

        public void Insert(T obj, AABB2 aabb)
        {
            Insert(obj, aabb, 0);
        }

        void Insert(T obj, AABB2 aabb, int nodeIndex)
        {
            AABB2 currentNodeAABB = _nodesAABBs[nodeIndex];
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
            _nodesAABBs.Add(new AABB2(center, size));
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
            AABB2 aabb = _nodesAABBs[nodeIndex];
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

        public AABB2 GetNodeAABB(int nodeIndex)
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
