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
        public class Node
        {
            public int _index;
            public int _parentIndex;
            public int[] _childrenIndices;
            public uint _depth;
            public Rectangle _rect;
            public List<T> _items;

            public bool IsRoot { get => _parentIndex < 0; }
            public bool IsLeaf { get => _depth == 0; }
            public bool HasChildren { get => _childrenIndices != null; }

            public bool Contains(Vector2 position)
            {
                bool containsX = (position.X > _rect.x) && (position.X < _rect.x + _rect.width);
                bool containsY = (position.Y > _rect.y) && (position.Y < _rect.y + _rect.width);
                return containsX && containsY;
            }

            public void Draw(Color color)
            {
                Vector2 cellPosition = new Vector2(_rect.x, _rect.y);
                Vector2 cellSize = new Vector2(_rect.width, _rect.height);
                Raylib.DrawRectangleLines((int)cellPosition.X, (int)cellPosition.Y, (int)cellSize.X, (int)cellSize.Y, color);
            }

        }
        private List<Node> _nodes;
        private uint _maxDepth;

        public List<Node> Nodes { get => _nodes; }
        public uint MaxDepth { get => _maxDepth; }

        public void Draw(Color color)
        {
            if (_nodes != null)
                for (int i = 0; i < _nodes.Count; i++)
                    _nodes[i].Draw(color);
        }

        public QuadTree(Vector2 center, float size, uint depth)
        {
            _maxDepth = depth;
            _nodes = new List<Node>();
            _nodes.Add(CreateNode(0, -1, center, size, depth));
        }

        public void Add(T obj, Vector2 position)
        {
            Node currentNode = _nodes[0];
            while (currentNode != null)
            {
            start:
                if (currentNode.Contains(position))
                {
                    if (currentNode._depth == 0)
                    {
                        currentNode._items.Add(obj);
                        return;
                    }

                    if (!currentNode.HasChildren)
                        SplitNode(currentNode);
                        
                    for (int i = 0; i < 4; i++)
                    {
                        Node childNode = _nodes[currentNode._childrenIndices[i]];
                        if (childNode.Contains(position))
                        {
                            currentNode = childNode;
                            goto start;
                        }
                    }
                    return;
                }
                else
                {
                    return;
                }
            }
        }

        Node CreateNode(int nodeIndex, int parentIndex, Vector2 center, float size, uint depth)
        {
            Vector2 rectPos = center - Vector2.One * size / 2;
            Vector2 rectSize = Vector2.One * size;
            Node node = new Node()
            {
                _index = nodeIndex,
                _parentIndex = parentIndex,
                _childrenIndices = null,
                _depth = depth,
                _rect = new Rectangle(rectPos.X, rectPos.Y, rectSize.X, rectSize.Y),
                _items = depth == 0 ? new List<T>() : null,
            };
            _nodes.Add(node);
            return node;
        }

        Node CreateNodeRecursive(int nodeIndex, int parentIndex, Vector2 center, float size, int depth)
        {
            Node node = CreateNode(nodeIndex, parentIndex, center, size, (uint)depth);
            if (depth > -1)
            {
                size /= 2;
                depth--;
                node._childrenIndices = new int[4];
                node._childrenIndices[0] = CreateNodeRecursive(_nodes.Count, parentIndex + 1, center + new Vector2(0.5f, 0.5f) * size, size, depth)._index;
                node._childrenIndices[1] = CreateNodeRecursive(_nodes.Count, parentIndex + 1, center + new Vector2(-0.5f, 0.5f) * size, size, depth)._index;
                node._childrenIndices[2] = CreateNodeRecursive(_nodes.Count, parentIndex + 1, center + new Vector2(-0.5f, -0.5f) * size, size, depth)._index;
                node._childrenIndices[3] = CreateNodeRecursive(_nodes.Count, parentIndex + 1, center + new Vector2(0.5f, -0.5f) * size, size, depth)._index;
            }
            return node;
        }

        void SplitNode(Node node)
        {
            Vector2 center = new Vector2(node._rect.x + node._rect.width / 2, node._rect.y + node._rect.height / 2);

            float size = (node._rect.width + node._rect.height) / 4;
            uint depth = node._depth - 1;

            node._childrenIndices ??= new int[4];
            node._childrenIndices[0] = CreateNode(_nodes.Count, node._index, center + new Vector2(0.5f, 0.5f) * size, size, depth)._index;
            node._childrenIndices[1] = CreateNode(_nodes.Count, node._index, center + new Vector2(-0.5f, 0.5f) * size, size, depth)._index;
            node._childrenIndices[2] = CreateNode(_nodes.Count, node._index, center + new Vector2(-0.5f, -0.5f) * size, size, depth)._index;
            node._childrenIndices[3] = CreateNode(_nodes.Count, node._index, center + new Vector2(0.5f, -0.5f) * size, size, depth)._index;

        }

        public Node GetNode(int cellIndex)
        {
            return _nodes[cellIndex];
        }

        public Node GetNode(Vector2 position, uint depth)
        {
            if (!_nodes[0].Contains(position))
                return null;
            return GetNode(_nodes[0], position, Math.Min(_maxDepth, depth));
        }

        Node GetNode(Node node, Vector2 position, uint depth)
        {
            if (!node.Contains(position))
                return null;

            if (node._depth == depth)
                return node;

            if (node._childrenIndices != null && node._childrenIndices.Length > 0)
                for (int i = 0; i < 4; i++)
                {
                    Node leafNode = GetNode(_nodes[node._childrenIndices[i]], position, depth);
                    if (leafNode != null) return leafNode;
                }

            return null;
        }
    }
}
