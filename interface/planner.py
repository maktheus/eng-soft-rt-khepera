"""Planejador de caminho A* sobre o mapa de ocupacao gerado pela interface.

O mapa (`grid.json`) e esparso: so tras celulas observadas, com `state` em
{free, visited, occupied}. Aqui:
  - `occupied` (dilatado pelo raio do robo) = bloqueado;
  - todo o resto (free, visited e o NAO observado) = livre (otimista) -- o
    Bug2 reativo do robo cuida de qualquer obstaculo nao mapeado.

Uso principal: `plan_waypoints(grid, start_xy, goal_xy)` -> lista de (x,y) em mm.
Funcao pura, sem dependencias externas, testavel offline.
"""
import heapq
import math

ROBOT_RADIUS_MM = 90.0   # ~meia base (105 mm) + folga; dilata os obstaculos
MARGIN_CELLS = 12        # folga do bounding box em celulas (espaco nao observado)


def _cell_of(x, y, r):
    return (int(math.floor(x / r)), int(math.floor(y / r)))


def _center(i, j, r):
    return ((i + 0.5) * r, (j + 0.5) * r)


def _blocked_set(grid):
    """Conjunto de celulas (i,j) bloqueadas = occupied dilatado pelo raio."""
    r = float(grid.get("resolution_mm", 50.0))
    occ = set()
    for c in grid.get("cells", []):
        if c.get("state") == "occupied":
            occ.add((int(c["i"]), int(c["j"])))
    if not occ:
        return set(), r
    infl = int(math.ceil(ROBOT_RADIUS_MM / r))
    blocked = set()
    for (i, j) in occ:
        for di in range(-infl, infl + 1):
            for dj in range(-infl, infl + 1):
                if di * di + dj * dj <= infl * infl:
                    blocked.add((i + di, j + dj))
    return blocked, r


def _reconstruct_path(target, start, came):
    if target == start:
        return [start]
    if target not in came:
        return []
    path = [target]
    while path[-1] != start:
        path.append(came[path[-1]])
    path.reverse()
    return path


def _simplify_cells(path, blocked, can_step):
    if len(path) < 3:
        return path

    # simplifica por linha-de-visada (Bresenham entre celulas)
    def visible(a, b):
        (x0, y0), (x1, y1) = a, b
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if (x, y) in blocked:
                return False
            if (x, y) == (x1, y1):
                return True
            px, py = x, y
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
            if not can_step((px, py), x - px, y - py):
                return False

    simp = [path[0]]
    k = 0
    while k < len(path) - 1:
        j = len(path) - 1
        while j > k + 1 and not visible(path[k], path[j]):
            j -= 1
        simp.append(path[j])
        k = j
    return simp


def _cells_to_waypoints(cells, r, start_xy, final_xy=None):
    if not cells:
        return []
    wps = [_center(i, j, r) for (i, j) in cells]
    if final_xy is not None:
        wps[-1] = (float(final_xy[0]), float(final_xy[1]))
    if len(wps) >= 2 and math.hypot(wps[0][0] - start_xy[0], wps[0][1] - start_xy[1]) < r:
        wps = wps[1:]   # descarta 1o waypoint coincidente com a pose atual
    elif len(wps) == 1 and math.hypot(wps[0][0] - start_xy[0], wps[0][1] - start_xy[1]) < r:
        wps = []
    return wps


def plan_attempt(grid, start_xy, goal_xy):
    """A* 8-conexo com diagnostico.

    Retorna:
      - ok=True e waypoints quando o alvo e alcancavel;
      - ok=False e attempted_waypoints com o melhor caminho parcial ate a
        celula explorada mais proxima do alvo quando nao existe caminho.
    """
    blocked, r = _blocked_set(grid)
    s = _cell_of(start_xy[0], start_xy[1], r)
    g = _cell_of(goal_xy[0], goal_xy[1], r)
    if s == g:
        return {
            "ok": True,
            "waypoints": [tuple(map(float, goal_xy))],
            "attempted_waypoints": [],
            "closest_cell": g,
            "closest_xy": tuple(map(float, goal_xy)),
            "explored_cells": 1,
        }

    # bounding box: envolve start, goal e celulas observadas, com folga
    xs = [s[0], g[0]]
    ys = [s[1], g[1]]
    for c in grid.get("cells", []):
        xs.append(int(c["i"])); ys.append(int(c["j"]))
    imin, imax = min(xs) - MARGIN_CELLS, max(xs) + MARGIN_CELLS
    jmin, jmax = min(ys) - MARGIN_CELLS, max(ys) + MARGIN_CELLS

    def inb(i, j):
        return imin <= i <= imax and jmin <= j <= jmax

    # o robo esta fisicamente na footprint do start: libera um disco em volta
    # dele (senao um obstaculo mapeado colado na origem o deixa "preso").
    infl = int(math.ceil(ROBOT_RADIUS_MM / r))
    free_start = {(s[0] + di, s[1] + dj)
                  for di in range(-infl, infl + 1)
                  for dj in range(-infl, infl + 1)
                  if di * di + dj * dj <= infl * infl}
    blocked = blocked - free_start

    neigh = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def can_step(cur, di, dj):
        nb = (cur[0] + di, cur[1] + dj)
        if not inb(*nb) or nb in blocked:
            return False
        # Nao corta quina: em diagonal, os dois vizinhos ortogonais precisam
        # estar livres para a footprint dilatada do robo passar com folga.
        if di and dj and ((cur[0] + di, cur[1]) in blocked or
                          (cur[0], cur[1] + dj) in blocked):
            return False
        return True

    openh = [(h(s, g), 0.0, s)]
    came = {}
    gscore = {s: 0.0}
    closed = set()
    best = s
    best_h = h(s, g)
    best_g = 0.0
    reached = False
    while openh:
        _, gc, cur = heapq.heappop(openh)
        if gc > gscore.get(cur, float("inf")):
            continue
        if cur == g:
            reached = True
            break
        if cur in closed:
            continue
        closed.add(cur)
        cur_h = h(cur, g)
        if cur_h < best_h or (cur_h == best_h and gc > best_g):
            best = cur
            best_h = cur_h
            best_g = gc
        for di, dj in neigh:
            nb = (cur[0] + di, cur[1] + dj)
            if nb in closed or not can_step(cur, di, dj):
                continue
            step = math.hypot(di, dj)
            ng = gc + step
            if ng < gscore.get(nb, float("inf")):
                gscore[nb] = ng
                came[nb] = cur
                heapq.heappush(openh, (ng + h(nb, g), ng, nb))

    explored = max(1, len(closed))
    if reached or g in came:
        path = _reconstruct_path(g, s, came)
        simp = _simplify_cells(path, blocked, can_step)
        return {
            "ok": True,
            "waypoints": _cells_to_waypoints(simp, r, start_xy, goal_xy),
            "attempted_waypoints": [],
            "closest_cell": g,
            "closest_xy": tuple(map(float, goal_xy)),
            "explored_cells": explored,
        }

    partial = _reconstruct_path(best, s, came)
    simp = _simplify_cells(partial, blocked, can_step)
    closest_xy = _center(best[0], best[1], r)
    return {
        "ok": False,
        "waypoints": [],
        "attempted_waypoints": _cells_to_waypoints(simp, r, start_xy, closest_xy),
        "closest_cell": best,
        "closest_xy": closest_xy,
        "distance_to_goal_mm": best_h * r,
        "explored_cells": explored,
    }


def plan_waypoints(grid, start_xy, goal_xy):
    """A* 8-conexo de start ate goal (mm). Retorna lista de waypoints (x,y) em mm,
    ja simplificada por linha-de-visada. Retorna [] se nao houver caminho."""
    result = plan_attempt(grid, start_xy, goal_xy)
    return result["waypoints"] if result.get("ok") else []
