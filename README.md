# Sokoban solver — sokoban-solver-01-run-20261121

One program (`main.ts`, Node 22, built-ins only) that answers all 5 instances
of a set in a single run, writing `/output/<id>/solution.txt` for each.

## Idea

A level decomposes into independent rooms off the one-wide hallway (the
player's row): every box stays in its room, so the level's push-optimal
count is the sum of its rooms' optima. The solver therefore

1. **Splits** the level at the hallway row into rooms (each room has exactly
   one door and as many goals as boxes).
2. **Solves each room push-optimally** with A\* over pushes. A state is the
   sorted box positions plus the player's reachable region (BFS); the
   heuristic is the minimum-cost box-to-goal assignment using BFS distances
   (exact for 3–4 boxes). Pruning: dead squares (corners and closed
   wall-adjacent runs with no goal) and freeze deadlocks (a box that cannot
   move on either axis). The player may stand anywhere in the hallway, since
   a box on a room's hallway-side row is pushed from there.
3. **Stitches** the room solutions together with BFS player walks through
   the hallway, in a fixed room order.

If a room's optimal search ever exceeded its work budget, the solver falls
back to weighted A\* so the instance still gets a valid (partial-credit)
answer instead of none; every instance always gets a `solution.txt`.

## Determinism

No clock is read anywhere. Every search is cut off by a fixed number of
node expansions, never by wall time, so two runs write identical bytes.

## Practice results

| instance | P\* | pushes | points |
|----------|----|--------|--------|
| i001 | 13 | 13 | 1000 |
| i002 | 16 | 16 | 1000 |
| i003 | 29 | 29 | 1000 |
| i004 | 47 | 47 | 1000 |
| i005 | 60 | 60 | 1000 |

Total: **1,000,000 / 1,000,000**, runtime ~1.8 s for the whole set
(budget: 60 s). The same score was reproduced on three freshly drawn sets
from the published generator (seeds other than the practice round).
