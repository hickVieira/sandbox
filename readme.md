## Implemented
- Verlet
    - substepping
    - edges/links/constraints
    - collisions
    - quadTree optimized
- Triangulation

## TODO
  - Verlet orientation (for 1 joint springs ?)
  - Verlet collision 'shapes' ?
    - Circle
    - Edge
    - Triangle
  - Convert some stuff to 3D

## Showcase
![planet](planet.gif)
![softbody](softbody.gif)
![quadtree](quadtree.gif)
![triangulators](triangulators.gif)

## Libraries
- [rayLib](https://www.raylib.com/) for rendering
- [my own custom Bowyer-Watson 2D delaunator](math/Delaunay.cs) mega hyper super duper criminally slow
- [delaunator-net](https://github.com/modios/delaunator-net) fast as hell
- [delaunator-sharp](https://github.com/nol1fe/delaunator-sharp) same
- [delaunator-csharp](https://github.com/wolktocs/delaunator-csharp) slightly slower (still hella fast)
- [s-hull](http://www.s-hull.org/) cs port of s-hull - it is slower than above
- [MIConvexHull](https://designengrlab.github.io/MIConvexHull/) not fast - however it is an n-dimensional solution?
- [Poly2Tri](https://github.com/Syncaidius/Poly2Tri) faster than original poly2tri-cs version - probably because it was ported from Java?
- [poly2tri-cs](https://github.com/Unity-Technologies/poly2tri-cs) not slow, not fast, no complains you know
- [Poly2Tri-biofluidix](https://github.com/BioFluidix/Poly2Tri) super slow - probably an early fork of the Java port?
- [LibTessDotNet](https://github.com/speps/LibTessDotNet) not fast, several times slower than modern Poly2Tri
- [QuickHull3D-biofluidix](https://github.com/BioFluidix/QuickHull3D) really fast - faster than MIConvexHull
- [QuickHull3D-oskar](https://github.com/OskarSigvardsson/unity-quickhull) slower than biofluidix's cs port - but faster than MIConvexHull
- [Delaunay-oskar](https://github.com/OskarSigvardsson/unity-delaunay) not fast
- [csDelaunay](https://github.com/PouletFrit/csDelaunay) not that fast - it exposes voronoi api only though, even though it states 'delaunay'
- [DelaunayVoronoi](https://github.com/RafaelKuebler/DelaunayVoronoi) slow and broken
- [Voronoi-Delaunay](https://github.com/IsaacGuan/Voronoi-Delaunay) not so slow but broken with bad precision issues

Obviously comparing some of the constrained delaunay vs non-constrained delaunay, or even 2D vs 3D vs nD isn't fair - so weight your judgement properly.

## License
FOR EDUCATIONAL PURPOSES ONLY. MANY EXTERNAL LIBRARIES IN USE (DIFFERENT LICENSES). CHECK THEIR SPECIFIC LICENSES.