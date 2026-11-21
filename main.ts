// Sokoban solver — written from scratch for sokoban-solver-01-run-20261121.
// Strategy: the level decomposes into independent rooms off a 1-wide hallway
// (the player's row). Each room is solved push-optimally with A* over pushes
// (state = box positions + player reachable region, min-assignment heuristic,
// dead-square + freeze-deadlock pruning), then the room solutions are stitched
// together with BFS player walks through the hallway. No clock is read anywhere;
// every search is cut off by a fixed number of node expansions, so two runs
// produce identical bytes.
import * as fs from "node:fs";
import * as path from "node:path";

const DIRS = [
  { dx: 0, dy: -1, ch: "u" },
  { dx: 0, dy: 1, ch: "d" },
  { dx: -1, dy: 0, ch: "l" },
  { dx: 1, dy: 0, ch: "r" },
];

type Grid = {
  w: number;
  h: number;
  wall: boolean[]; // index y*w+x
  goals: Set<number>;
  boxes: number[];
  player: number;
};

function parseLevel(text: string): Grid {
  const lines = text.replace(/\r/g, "").split("\n");
  while (lines.length > 0 && lines[lines.length - 1] === "") lines.pop();
  const h = lines.length;
  let w = 0;
  for (const l of lines) w = Math.max(w, l.length);
  const wall: boolean[] = new Array(w * h).fill(true);
  const goals = new Set<number>();
  const boxes: number[] = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const ch = x < lines[y].length ? lines[y][x] : " ";
      const i = y * w + x;
      if (ch === "#") { wall[i] = true; continue; }
      wall[i] = false;
      if (ch === "." || ch === "*" || ch === "+") goals.add(i);
      if (ch === "$" || ch === "*") boxes.push(i);
      if (ch === "@" || ch === "+") player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

function xy(i: number, w: number): [number, number] { return [i % w, Math.floor(i / w)]; }

// BFS shortest path (as move string) from `from` to `to` over floor cells,
// treating every cell in `blocked` as impassable. Deterministic neighbour order.
function bfsPath(g: Grid, from: number, to: number, blocked: Set<number>): string | null {
  if (from === to) return "";
  const prev = new Map<number, number>();
  const prevCh = new Map<number, string>();
  const q: number[] = [from];
  prev.set(from, -1);
  let head = 0;
  while (head < q.length) {
    const cur = q[head++];
    const [cx, cy] = xy(cur, g.w);
    for (const d of DIRS) {
      const nx = cx + d.dx, ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) continue;
      const ni = ny * g.w + nx;
      if (g.wall[ni] || blocked.has(ni) || prev.has(ni)) continue;
      prev.set(ni, cur);
      prevCh.set(ni, d.ch);
      if (ni === to) {
        let s = "";
        let c = ni;
        while (c !== from) { s = prevCh.get(c)! + s; c = prev.get(c)!; }
        return s;
      }
      q.push(ni);
    }
  }
  return null;
}

// Reachable cells from `start` over `allowed` cells minus `boxSet`.
// Returns a BigInt bitmask over the index of each cell in `allowedList`.
function reachMask(
  allowedIdx: Map<number, number>, allowedList: number[], g: Grid,
  start: number, boxSet: Set<number>
): bigint {
  let mask = 0n;
  const si = allowedIdx.get(start);
  if (si === undefined) return mask;
  const q: number[] = [start];
  mask |= 1n << BigInt(si);
  let head = 0;
  while (head < q.length) {
    const cur = q[head++];
    const [cx, cy] = xy(cur, g.w);
    for (const d of DIRS) {
      const nx = cx + d.dx, ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) continue;
      const ni = ny * g.w + nx;
      if (boxSet.has(ni)) continue;
      const ai = allowedIdx.get(ni);
      if (ai === undefined) continue;
      const bit = 1n << BigInt(ai);
      if ((mask & bit) !== 0n) continue;
      mask |= bit;
      q.push(ni);
    }
  }
  return mask;
}

// Minimal binary heap with deterministic tie-breaking by insertion sequence.
class Heap {
  a: { f: number; seq: number; id: number }[] = [];
  get size(): number { return this.a.length; }
  push(f: number, seq: number, id: number): void {
    const h = this.a;
    h.push({ f, seq, id });
    let i = h.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (h[p].f < h[i].f || (h[p].f === h[i].f && h[p].seq <= h[i].seq)) break;
      const t = h[p]; h[p] = h[i]; h[i] = t;
      i = p;
    }
  }
  pop(): { f: number; seq: number; id: number } {
    const h = this.a;
    const top = h[0];
    const last = h.pop()!;
    if (h.length > 0) {
      h[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1, r = l + 1;
        let m = i;
        if (l < h.length && (h[l].f < h[m].f || (h[l].f === h[m].f && h[l].seq < h[m].seq))) m = l;
        if (r < h.length && (h[r].f < h[m].f || (h[r].f === h[m].f && h[r].seq < h[m].seq))) m = r;
        if (m === i) break;
        const t = h[m]; h[m] = h[i]; h[i] = t;
        i = m;
      }
    }
    return top;
  }
}

type Room = {
  cells: number[];      // room floor cells (door cell included)
  boxes: number[];      // initial box cells inside the room
  goals: number[];      // goal cells inside the room
  door: number;         // room cell adjacent to the hallway
  entry: number;        // hallway cell adjacent to the door
};

// Split the floor (excluding the hallway row) into connected components.
// Returns null when the split does not cover every box exactly once with a
// matching number of goals and exactly one door per component.
function detectRooms(g: Grid): Room[] | null {
  const hy = Math.floor(g.player / g.w);
  const seen = new Array(g.w * g.h).fill(false);
  const rooms: Room[] = [];
  const coveredBoxes = new Set<number>();
  for (let y = 0; y < g.h; y++) {
    for (let x = 0; x < g.w; x++) {
      const i = y * g.w + x;
      if (y === hy || g.wall[i] || seen[i]) continue;
      const cells: number[] = [];
      const q: number[] = [i];
      seen[i] = true;
      let head = 0;
      while (head < q.length) {
        const cur = q[head++];
        cells.push(cur);
        const [cx, cy] = xy(cur, g.w);
        for (const d of DIRS) {
          const nx = cx + d.dx, ny = cy + d.dy;
          if (ny === hy || nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) continue;
          const ni = ny * g.w + nx;
          if (g.wall[ni] || seen[ni]) continue;
          seen[ni] = true;
          q.push(ni);
        }
      }
      const boxes = cells.filter((c) => g.boxes.includes(c));
      const goals = cells.filter((c) => g.goals.has(c));
      const doors = cells.filter((c) => {
        const [cx, cy] = xy(c, g.w);
        return Math.abs(cy - hy) === 1 && !g.wall[hy * g.w + cx];
      });
      if (boxes.length === 0 || boxes.length !== goals.length || doors.length !== 1) return null;
      for (const b of boxes) {
        if (coveredBoxes.has(b)) return null;
        coveredBoxes.add(b);
      }
      const door = doors[0];
      const [dx, dy] = xy(door, g.w);
      void dy;
      rooms.push({ cells, boxes: boxes.slice().sort((a, b) => a - b), goals, door, entry: hy * g.w + dx });
    }
  }
  if (coveredBoxes.size !== g.boxes.length) return null;
  // Deterministic order: rooms above the hallway first (top to bottom,
  // left to right), then rooms below.
  rooms.sort((a, b) => {
    const ay = Math.floor(a.door / g.w), by = Math.floor(b.door / g.w);
    const aAbove = ay < hy ? 0 : 1, bAbove = by < hy ? 0 : 1;
    if (aAbove !== bAbove) return aAbove - bAbove;
    if (ay !== by) return ay - by;
    return (a.door % g.w) - (b.door % g.w);
  });
  return rooms;
}

// ---- Room solver: push-optimal A* ---------------------------------------
// Precompute cells a box can never leave (unless the cell is a goal):
// corners and wall-adjacent runs that contain no goal.
function deadSquares(g: Grid, allowed: Set<number>, roomCells: number[]): Set<number> {
  const dead = new Set<number>();
  const blocked = (i: number): boolean => !allowed.has(i);
  for (const c of roomCells) {
    if (g.goals.has(c)) continue;
    const [x, y] = xy(c, g.w);
    const up = y - 1 < 0 ? -1 : (y - 1) * g.w + x;
    const dn = y + 1 >= g.h ? -1 : (y + 1) * g.w + x;
    const lf = x - 1 < 0 ? -1 : y * g.w + (x - 1);
    const rt = x + 1 >= g.w ? -1 : y * g.w + (x + 1);
    const bUp = up === -1 || blocked(up);
    const bDn = dn === -1 || blocked(dn);
    const bLf = lf === -1 || blocked(lf);
    const bRt = rt === -1 || blocked(rt);
    if ((bUp && bLf) || (bUp && bRt) || (bDn && bLf) || (bDn && bRt)) dead.add(c);
  }
  // Wall-adjacent runs with no goal: scan rows (wall above / below) and
  // columns (wall left / right). A run is only dead when a box inside it
  // cannot leave it either: both cells just past the run's ends (along the
  // run) must be blocked too, otherwise the box can slide out of the run
  // and then be pushed away from the wall.
  const scanRuns = (cells: number[], alongWall: (c: number) => boolean, stepSize: number): void => {
    let run: number[] = [];
    const flush = (): void => {
      if (run.length > 0 && !run.some((c) => g.goals.has(c))) {
        const [fx, fy] = xy(run[0], g.w);
        const [lx, ly] = xy(run[run.length - 1], g.w);
        let startBlocked: boolean;
        let endBlocked: boolean;
        if (stepSize === 1) {
          startBlocked = fx === 0 || !allowed.has(run[0] - 1);
          endBlocked = lx === g.w - 1 || !allowed.has(run[run.length - 1] + 1);
        } else {
          startBlocked = fy === 0 || !allowed.has(run[0] - g.w);
          endBlocked = ly === g.h - 1 || !allowed.has(run[run.length - 1] + g.w);
        }
        if (startBlocked && endBlocked) {
          for (const c of run) if (!g.goals.has(c)) dead.add(c);
        }
      }
      run = [];
    };
    let prev = -1000000;
    for (const c of cells) {
      if (!alongWall(c) || c !== prev + stepSize) flush();
      if (alongWall(c)) { run.push(c); prev = c; } else prev = -1000000;
    }
    flush();
  };
  for (let y = 0; y < g.h; y++) {
    const row: number[] = [];
    for (let x = 0; x < g.w; x++) { const i = y * g.w + x; if (allowed.has(i)) row.push(i); }
    scanRuns(row, (c) => { const [x, cy] = xy(c, g.w); return cy - 1 < 0 || blocked((cy - 1) * g.w + x); }, 1);
    scanRuns(row, (c) => { const [x, cy] = xy(c, g.w); return cy + 1 >= g.h || blocked((cy + 1) * g.w + x); }, 1);
  }
  for (let x = 0; x < g.w; x++) {
    const col: number[] = [];
    for (let y = 0; y < g.h; y++) { const i = y * g.w + x; if (allowed.has(i)) col.push(i); }
    scanRuns(col, (c) => { const [cx, cy] = xy(c, g.w); return cx - 1 < 0 || blocked(cy * g.w + (cx - 1)); }, g.w);
    scanRuns(col, (c) => { const [cx, cy] = xy(c, g.w); return cx + 1 >= g.w || blocked(cy * g.w + (cx + 1)); }, g.w);
  }
  return dead;
}

// All permutations of [0..n).
function permutations(n: number): number[][] {
  const out: number[][] = [];
  const a: number[] = [];
  for (let i = 0; i < n; i++) a.push(i);
  const gen = (k: number): void => {
    if (k === n) { out.push(a.slice()); return; }
    for (let i = k; i < n; i++) {
      const t = a[k]; a[k] = a[i]; a[i] = t;
      gen(k + 1);
      const t2 = a[k]; a[k] = a[i]; a[i] = t2;
    }
  };
  gen(0);
  return out;
}

type Push = { box: number; dir: number }; // dir = index into DIRS

// Solve one room push-optimally. Returns the push list, or null.
// `requireExit`: goal states must also let the player reach the door cell.
// `budget`: max node expansions (work-based cutoff, deterministic).
// `weight`: 1 = optimal A*, >1 = weighted (finds a solution faster).
function solveRoom(
  g: Grid, room: Room, requireExit: boolean, budget: number, weight: number
): Push[] | null {
  // Walkable cells for the player: the room plus the whole hallway row
  // (a box on the room's hallway-side row is pushed from the hallway).
  // Boxes themselves may only ever occupy room cells.
  const hy = Math.floor(room.entry / g.w);
  const walkList = room.cells.slice();
  for (let x = 0; x < g.w; x++) {
    const i = hy * g.w + x;
    if (!g.wall[i] && !walkList.includes(i)) walkList.push(i);
  }
  const roomSet = new Set<number>(room.cells);
  const allowedList = walkList;
  const allowed = new Set<number>(allowedList);
  const allowedIdx = new Map<number, number>();
  allowedList.forEach((c, i) => allowedIdx.set(c, i));
  const dead = deadSquares(g, allowed, room.cells);
  // All-pairs BFS distances over room cells only (boxes never leave the
  // room, so this stays a tight admissible bound).
  const dist = new Map<number, Map<number, number>>();
  for (const s of room.cells) {
    const d = new Map<number, number>();
    d.set(s, 0);
    const q: number[] = [s];
    let head = 0;
    while (head < q.length) {
      const cur = q[head++];
      const [cx, cy] = xy(cur, g.w);
      for (const dd of DIRS) {
        const nx = cx + dd.dx, ny = cy + dd.dy;
        if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) continue;
        const ni = ny * g.w + nx;
        if (!roomSet.has(ni) || d.has(ni)) continue;
        d.set(ni, d.get(cur)! + 1);
        q.push(ni);
      }
    }
    dist.set(s, d);
  }
  const goals = room.goals;
  const perms = permutations(goals.length);
  const heuristic = (boxes: number[]): number => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      let ok = true;
      for (let i = 0; i < boxes.length; i++) {
        const d = dist.get(boxes[i])!.get(goals[p[i]]);
        if (d === undefined) { ok = false; break; }
        s += d;
      }
      if (ok && s < best) best = s;
    }
    return best;
  };
  const goalSet = new Set<number>(goals);
  const isGoalState = (boxes: number[]): boolean => boxes.every((b) => goalSet.has(b));
  // Freeze deadlock: a box not on a goal that is blocked on both sides of
  // an axis by walls/out-of-room cells or other frozen boxes can never move.
  const frozenDead = (boxes: number[]): boolean => {
    const boxSet = new Set<number>(boxes);
    const frozen = new Set<number>();
    let changed = true;
    while (changed) {
      changed = false;
      for (const b of boxes) {
        if (frozen.has(b) || goalSet.has(b)) continue;
        const [x, y] = xy(b, g.w);
        const blk = (nx: number, ny: number): boolean => {
          if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) return true;
          const ni = ny * g.w + nx;
          if (!allowed.has(ni)) return true;
          return boxSet.has(ni) && frozen.has(ni);
        };
        // A box is frozen only when it cannot move on EITHER axis: both
        // horizontal neighbours and both vertical neighbours blocked
        // (by a wall, an out-of-room cell, or another frozen box).
        if ((blk(x - 1, y) && blk(x + 1, y)) && (blk(x, y - 1) && blk(x, y + 1))) {
          frozen.add(b);
          changed = true;
        }
      }
    }
    for (const b of frozen) if (!goalSet.has(b)) return true;
    return false;
  };

  const startBoxes = room.boxes.slice().sort((a, b) => a - b);
  const startMask = reachMask(allowedIdx, allowedList, g, room.entry, new Set<number>(startBoxes));
  const startH = heuristic(startBoxes);
  if (startH === Infinity) return null;

  const statesBoxes: number[][] = [startBoxes];
  const statesPlayer: number[] = [room.entry];
  const statesG: number[] = [0];
  const statesParent: number[] = [-1];
  const statesPush: (Push | null)[] = [null];
  const seen = new Map<string, number>();
  seen.set(startBoxes.join(",") + "|" + startMask.toString(36), 0);
  const heap = new Heap();
  let seq = 0;
  heap.push(startH * weight, seq++, 0);
  let expansions = 0;
  while (heap.size > 0) {
    if (expansions >= budget) return null;
    const { id } = heap.pop();
    const boxes = statesBoxes[id];
    const gScore = statesG[id];
    expansions++;
    const boxSet = new Set<number>(boxes);
    const mask = reachMask(allowedIdx, allowedList, g, statesPlayer[id], boxSet);
    if (isGoalState(boxes)) {
      if (!requireExit || (mask & (1n << BigInt(allowedIdx.get(room.door)!))) !== 0n) {
        const pushes: Push[] = [];
        let cur = id;
        while (statesParent[cur] !== -1) {
          pushes.push(statesPush[cur]!);
          cur = statesParent[cur];
        }
        pushes.reverse();
        return pushes;
      }
    }
    // Generate pushes: for each box, each direction, player must reach the
    // cell behind the box and the destination must be a free allowed cell.
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      const [bx, by] = xy(b, g.w);
      for (let di = 0; di < 4; di++) {
        const d = DIRS[di];
        const dest = (by + d.dy) * g.w + (bx + d.dx);
        if (!roomSet.has(dest) || boxSet.has(dest)) continue;
        if (dead.has(dest) && !goalSet.has(dest)) continue;
        const behind = (by - d.dy) * g.w + (bx - d.dx);
        const behindIdx = allowedIdx.get(behind);
        if (behindIdx === undefined) continue;
        if ((mask & (1n << BigInt(behindIdx))) === 0n) continue;
        const nb = boxes.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        const nbSet = new Set<number>(nb);
        if (frozenDead(nb)) continue;
        const h = heuristic(nb);
        if (h === Infinity) continue;
        const nmask = reachMask(allowedIdx, allowedList, g, b, nbSet);
        const key = nb.join(",") + "|" + nmask.toString(36);
        const prev = seen.get(key);
        if (prev !== undefined && statesG[prev] <= gScore + 1) continue;
        const nid = statesBoxes.length;
        statesBoxes.push(nb);
        statesPlayer.push(b);
        statesG.push(gScore + 1);
        statesParent.push(id);
        statesPush.push({ box: b, dir: di });
        seen.set(key, nid);
        heap.push((gScore + 1) + weight * h, seq++, nid);
      }
    }
  }
  return null;
}

// ---- Stitching ----------------------------------------------------------
// Convert a room's push list into player moves, walking from `playerPos`
// (updated in place via the returned end position) with all current boxes
// as obstacles. Returns the move string, or null if a walk is impossible.
function stitchPushes(
  g: Grid, pushes: Push[], playerPos: number, boxSet: Set<number>
): { moves: string; endPlayer: number } | null {
  let moves = "";
  let cur = playerPos;
  for (const p of pushes) {
    const [bx, by] = xy(p.box, g.w);
    const d = DIRS[p.dir];
    const behind = (by - d.dy) * g.w + (bx - d.dx);
    const dest = (by + d.dy) * g.w + (bx + d.dx);
    const walk = bfsPath(g, cur, behind, boxSet);
    if (walk === null) return null;
    moves += walk;
    moves += d.ch;
    boxSet.delete(p.box);
    boxSet.add(dest);
    cur = p.box;
  }
  return { moves, endPlayer: cur };
}

function solveLevel(g: Grid): string {
  const rooms = detectRooms(g);
  let moves = "";
  let playerPos = g.player;
  const boxSet = new Set<number>(g.boxes);
  if (rooms !== null) {
    for (let ri = 0; ri < rooms.length; ri++) {
      const room = rooms[ri];
      const last = ri === rooms.length - 1;
      let pushes = solveRoom(g, room, false, 120000, 1);
      if (pushes !== null && !last) {
        // Check the player can leave the room afterwards; if not, re-solve
        // requiring an exitable final state.
        const tmpSet = new Set<number>(boxSet);
        const st = stitchPushes(g, pushes, playerPos, tmpSet);
        if (st !== null) {
          const exitWalk = bfsPath(g, st.endPlayer, room.entry, tmpSet);
          if (exitWalk === null) {
            const alt = solveRoom(g, room, true, 120000, 1);
            if (alt !== null) pushes = alt;
          }
        }
      }
      if (pushes === null) pushes = solveRoom(g, room, false, 120000, 2);
      if (pushes === null) pushes = solveRoom(g, room, false, 200000, 4);
      if (pushes === null) return moves; // keep a valid (partial) answer
      const st = stitchPushes(g, pushes, playerPos, boxSet);
      if (st === null) return moves;
      moves += st.moves;
      playerPos = st.endPlayer;
    }
    return moves;
  }
  // Fallback: whole level as one room (player start as entry).
  const whole: Room = {
    cells: g.boxes.length >= 0 ? allFloor(g) : [],
    boxes: g.boxes.slice(),
    goals: [...g.goals],
    door: g.player,
    entry: g.player,
  };
  let pushes = solveRoom(g, whole, false, 120000, 1);
  if (pushes === null) pushes = solveRoom(g, whole, false, 120000, 3);
  if (pushes === null) return "";
  const st = stitchPushes(g, pushes, playerPos, boxSet);
  return st === null ? "" : st.moves;
}

function allFloor(g: Grid): number[] {
  const out: number[] = [];
  for (let i = 0; i < g.w * g.h; i++) if (!g.wall[i]) out.push(i);
  return out;
}

function main(): void {
  const inputDir = process.argv[2] ?? process.env.MAKEFUN_INPUT_DIR ?? "/input";
  const outputDir = process.argv[3] ?? process.env.MAKEFUN_OUTPUT_DIR ?? "/output";
  const ids = ["i001", "i002", "i003", "i004", "i005"];
  for (const id of ids) {
    let solution = "";
    try {
      const text = fs.readFileSync(path.join(inputDir, id, "level.txt"), "utf8");
      const grid = parseLevel(text);
      solution = solveLevel(grid);
    } catch {
      solution = "";
    }
    const dir = path.join(outputDir, id);
    fs.mkdirSync(dir, { recursive: true });
    fs.writeFileSync(path.join(dir, "solution.txt"), solution, "utf8");
  }
}

main();
