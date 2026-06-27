from __future__ import annotations
import collections
from warehouse_api import Action, CellType, Observation, Position

# --- MEMORIA GLOBAL ---
# Guardamos el plan (ruta) de cada robot.
_PLANS: dict[int, list[Position]] = {}
# Contador de cuántos ticks lleva atascado un robot intentando moverse a la misma celda.
_STUCK_TICKS: dict[int, int] = {}

def create_layout() -> dict[str, object]:
    """Genera un layout válido basado en bloques estándar (960 estantes)."""
    shelves: list[list[int]] = []
    for x0 in range(3, 48, 4):
        for y0, y1 in ((3, 12), (15, 24), (27, 36), (39, 48)):
            for x in (x0, x0 + 1):
                for y in range(y0, y1 + 1):
                    shelves.append([x, y])
    return {"schema_version": 1, "shelves": shelves}

def act(observation: Observation) -> Action:
    rid = observation.robot_id
    pos = observation.position

    # 1. Comprobar si podemos realizar acciones de recogida/entrega
    if not observation.carrying_item and _is_adjacent(pos, observation.target_item_position):
        _PLANS[rid] = []  # Limpiamos el plan tras llegar
        return Action.PICKUP

    drop_cell = _get_drop_cell(observation.base_position)
    if observation.carrying_item and pos == drop_cell:
        _PLANS[rid] = []  # Limpiamos el plan tras llegar
        return Action.DROP

    # 2. Recuperar el plan actual o crear uno nuevo
    plan = _PLANS.get(rid, [])

    if not plan:
        # Definir objetivos:
        # - Si lleva caja -> ir a su drop_cell
        # - Si NO lleva caja -> ir a cualquier celda adyacente vacía de su target
        goal_cells = set()
        if observation.carrying_item:
            goal_cells.add(drop_cell)
        else:
            goal_cells = _get_adjacent_empty(observation, observation.target_item_position)

        plan = _bfs(pos, goal_cells, observation.grid)
        _PLANS[rid] = plan
        _STUCK_TICKS[rid] = 0

    if not plan:
        return Action.WAIT  # Objetivo inalcanzable temporalmente

    next_pos = plan[0]

    # 3. Comprobar colisiones inmediatas
    occupied = set(observation.all_robot_positions.values())
    if next_pos in occupied:
        stuck = _STUCK_TICKS.get(rid, 0) + 1
        _STUCK_TICKS[rid] = stuck
        
        if stuck > 3: 
            # Si llevamos > 3 ticks atascados frente a otro robot, recalculamos
            # la ruta considerando su posición actual como un obstáculo.
            goal_cells = set([drop_cell]) if observation.carrying_item else _get_adjacent_empty(observation, observation.target_item_position)
            new_plan = _bfs(pos, goal_cells, observation.grid, avoid={next_pos})
            if new_plan:
                _PLANS[rid] = new_plan
                _STUCK_TICKS[rid] = 0
            
        return Action.WAIT

    # 4. Movernos
    _PLANS[rid] = plan[1:]  # Quitamos el paso actual del plan
    _STUCK_TICKS[rid] = 0
    return _get_action_for_move(pos, next_pos)


def _is_adjacent(a: Position, b: Position) -> bool:
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


def _get_drop_cell(base: Position) -> Position:
    x, y = base
    if y == 0: return (x, 1)
    if y == 51: return (x, 50)
    if x == 0: return (1, y)
    return (50, y)


def _get_adjacent_empty(obs: Observation, pos: Position) -> set[Position]:
    tx, ty = pos
    candidates = [(tx+1, ty), (tx-1, ty), (tx, ty+1), (tx, ty-1)]
    valid = set()
    for cx, cy in candidates:
        if 0 <= cy < len(obs.grid) and 0 <= cx < len(obs.grid[cy]):
            if obs.grid[cy][cx] == CellType.EMPTY:
                valid.add((cx, cy))
    return valid


def _bfs(start: Position, goals: set[Position], grid: tuple[tuple[CellType, ...], ...], avoid: set[Position] | None = None) -> list[Position]:
    """Búsqueda en anchura (BFS) para encontrar el camino más corto a uno de los objetivos."""
    if start in goals:
        return []
    
    avoid_set = avoid or set()
    queue = collections.deque([(start, [])])
    visited = {start} | avoid_set

    while queue:
        curr, path = queue.popleft()
        cx, cy = curr

        for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nx, ny = cx + dx, cy + dy
            if 0 <= ny < len(grid) and 0 <= nx < len(grid[ny]):
                if grid[ny][nx] == CellType.EMPTY and (nx, ny) not in visited:
                    npos = (nx, ny)
                    npath = path + [npos]
                    if npos in goals:
                        return npath
                    visited.add(npos)
                    queue.append((npos, npath))
    return []


def _get_action_for_move(curr: Position, nxt: Position) -> Action:
    """Convierte una coordenada actual y la siguiente en una Action direccional."""
    cx, cy = curr
    nx, ny = nxt
    if nx > cx: return Action.RIGHT
    if nx < cx: return Action.LEFT
    if ny > cy: return Action.DOWN
    if ny < cy: return Action.UP
    return Action.WAIT
