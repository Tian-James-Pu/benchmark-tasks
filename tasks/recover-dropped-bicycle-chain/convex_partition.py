"""Merge adjacent triangles without changing the authored planar solid.

Concave tooth seats and the central bore remain outside every convex piece.
The cap keeps extruded convex meshes small enough to avoid cooking reduction.
"""
import heapq
import math
from shapely import constrained_delaunay_triangles, union_all


def partition(polygon, maximum_vertices=24, area_tolerance=1e-15):
    pieces={i:g for i,g in enumerate(constrained_delaunay_triangles(polygon).geoms)}
    initial_count=len(pieces)
    edges={}
    for i,p in pieces.items():
        coordinates=list(p.exterior.coords)
        for a,b in zip(coordinates[:-1],coordinates[1:]):
            key=tuple(sorted((a,b)))
            edges.setdefault(key,[]).append(i)
    neighbors={i:set() for i in pieces}
    heap=[]
    for (a,b),ids in edges.items():
        if len(ids)==2:
            i,j=ids;neighbors[i].add(j);neighbors[j].add(i)
            heapq.heappush(heap,(-math.dist(a,b),i,j))
        elif len(ids)>2:
            raise ValueError('Nonmanifold triangulation edge')
    next_id=initial_count
    while heap:
        _,i,j=heapq.heappop(heap)
        if i not in pieces or j not in pieces or j not in neighbors[i]:
            continue
        combined=pieces[i].union(pieces[j])
        if combined.geom_type!='Polygon' or combined.interiors:
            continue
        hull=combined.convex_hull
        if hull.area-combined.area>area_tolerance or len(hull.exterior.coords)-1>maximum_vertices:
            continue
        if hull.symmetric_difference(combined).area>area_tolerance:
            raise ValueError('Convex merge would change a contact boundary')
        adjacent=(neighbors[i]|neighbors[j])-{i,j}
        k=next_id;next_id+=1
        del pieces[i],pieces[j]
        del neighbors[i],neighbors[j]
        pieces[k]=hull;neighbors[k]=adjacent
        for other in adjacent:
            neighbors[other].discard(i);neighbors[other].discard(j);neighbors[other].add(k)
            length=hull.boundary.intersection(pieces[other].boundary).length
            heapq.heappush(heap,(-length,k,other))
    result=list(pieces.values())
    reconstructed=union_all(result)
    difference=reconstructed.symmetric_difference(polygon).area
    if difference>max(area_tolerance*initial_count,1e-13):
        raise ValueError('Convex partition changed sprocket union or bore')
    if abs(sum(p.area for p in result)-polygon.area)>max(area_tolerance*initial_count,1e-13):
        raise ValueError('Convex partition pieces have area overlap')
    return result,dict(triangle_count=initial_count,convex_piece_count=len(result),
                       union_symmetric_difference_area_m2=difference,
                       maximum_vertices=max(len(p.exterior.coords)-1 for p in result))
