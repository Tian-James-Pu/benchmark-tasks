"""Read-only dynamic contact paths from an assembly to any robot body.

Fixed supports are terminal boundaries: objects sharing a bench are not thereby
robot-connected. Contact edges must come from measured native forces; authored
unbreakable joints supply the separate mechanical edges.
"""
import math


def robot_contact_path(*, assembly, dynamic, robots, fixed, mechanical_edges, contacts, epsilon=1e-4):
    assembly=set(assembly);dynamic=set(dynamic);robots=set(robots);fixed=set(fixed)
    if not assembly or not assembly<=dynamic or dynamic&robots or dynamic&fixed or robots&fixed:
        raise ValueError('Invalid contact graph body sets')
    if not math.isfinite(epsilon) or epsilon<0:raise ValueError('Invalid contact tolerance')
    graph={p:set() for p in dynamic};robot_edges={p:set() for p in dynamic}
    for a,b in mechanical_edges:
        if a not in dynamic or b not in dynamic or a==b:raise ValueError('Invalid mechanical edge')
        graph[a].add(b);graph[b].add(a)
    for a,b,force in contacts:
        if a not in dynamic or b not in dynamic|robots|fixed or not math.isfinite(force) or force<0:
            raise ValueError('Invalid measured contact edge')
        if force<=epsilon or a==b:continue
        if b in dynamic:graph[a].add(b);graph[b].add(a)
        elif b in robots:robot_edges[a].add(b)
        # Fixed supports never propagate robot connectivity.
    todo=[(a,(a,)) for a in sorted(assembly)];seen=set(assembly)
    for node,path in todo:
        if robot_edges[node]:return path+(min(robot_edges[node]),)
        for other in sorted(graph[node]-seen):
            seen.add(other);todo.append((other,path+(other,)))
    return None
