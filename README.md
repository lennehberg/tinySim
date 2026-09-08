# TinySim2D

A small 2D rigid-body physics simulator, written as a learning exercise for a
larger robotics software stack. It advances bodies through time under gravity
and applied forces — both linear and angular motion — and draws them with SFML.

See `TinySim2D Spec Sheet.md` for scope, invariants and acceptance criteria.

## Requirements

- A C++20 compiler (`clang++`, as shipped with the Xcode Command Line Tools)
- [SFML 3](https://www.sfml-dev.org/) — the renderer uses the SFML 3 API and
  will not build against SFML 2
- `make`

On macOS with Homebrew:

```sh
brew install sfml
```

The Makefile locates SFML with `brew --prefix sfml`, so it works on both Apple
Silicon (`/opt/homebrew`) and Intel (`/usr/local`) without editing paths.

## Building and running

| Command | What it does |
| --- | --- |
| `make` / `make build` | Build the simulator into `./tinysim` |
| `make test` | Build and run the unit test suite |
| `make visual` | Build and run the visual check harness |
| `make clean` | Remove all binaries and debug bundles |

> **Note:** `src/main.cpp` is currently an empty stub, so `make build` fails at
> the link stage with `Undefined symbols: _main`. `make test` and `make visual`
> both work — they have their own entry points. Writing `main.cpp` against the
> current `Renderer` API is the next step.

## Tests

```sh
make test
```

105 tests, no external test framework — `tests/test_utils.h` is a ~100 line
harness where tests self-register via a `TEST(suite, name)` macro.

| Suite | Tests | Covers |
| --- | --- | --- |
| `Vec2` | 27 | vector arithmetic, indexing, `dot`/`cross`/`d2` |
| `RigidBody` | 32 | construction, forces and torque, integration, shape ownership |
| `World` | 17 | body creation, gravity, force clearing |
| `Renderer` | 23 | coordinate/angle transforms, null-safety guards |
| `SpecAcceptance` | 6 | the acceptance tests from the spec sheet |

The `SpecAcceptance` suite maps directly onto section 10 of the spec sheet:

- **Test A** — no forces: a body at `(0,0)` moving at `(1,0)` is at `(2,0)`
  after 2 s, still moving at `(1,0)`. Checked at both `RigidBody` and `World`
  level.
- **Test B** — gravity: a body starting at rest develops negative vertical
  velocity.
- **Test C** — force through the centre of mass: linear acceleration, and
  angular velocity stays exactly zero.
- **Test D** — force offset from the centre of mass: both linear *and* angular
  acceleration.

Adding a test file means dropping it in `tests/` and adding it to `TEST_SRCS`
in the Makefile. Tests never open a window, so they run headless.

## Visual check

```sh
make visual
```

Opens a window and cycles through 11 scenes, looping until you close it. Each
scene prints its name and what to look for, so you can tell whether what you
see is correct. This is a *human* check — it makes no assertions and is not
part of `make test`.

Scenes: free fall, projectile, spin in place, tumbling projectile, off-centre
force, mass independence, circle vs box, spin right, spin both ways, tumbling
right, opposing torques.

A positive angular velocity spins **counter-clockwise** (visually left); a
negative one spins clockwise (right).

## Layout

```
include/     public headers
  Vec2.h         2D vector maths
  Shape.h        abstract shape + ShapeType enum
  BoxShape.h     rectangle, inertia = 1/12 m (w^2 + h^2)
  CircleShape.h  circle, inertia = 1/2 m r^2
  RigidBody.h    physical state; owns its Shape
  World.h        owns bodies, applies gravity, advances the simulation
  Renderer.h     SFML window and drawing
src/         implementations + main.cpp
tests/       unit tests and the visual harness
```

The physics core (`Vec2`, `Shape`, `RigidBody`, `World`) has no SFML
dependency; only `Renderer` includes SFML. The renderer is the only place that
knows about pixels — physics is in metres with **+y pointing up**, and
`Renderer::worldToScreen` converts to SFML's y-down pixel space.

## Conventions

- Metres, kilograms, seconds; angles in radians
- `mass > 0` means a dynamic body; `mass == 0` means a static one, which
  gravity and integration both skip
- Forces and torques accumulate during a frame and are cleared by
  `World::step()`
- Integration is semi-implicit Euler: velocity is updated first, then position
  uses the new velocity

## Not implemented yet

- Collision detection and response — bodies pass straight through each other
  and through static bodies
- `World::createBody()` takes no shape; attach one afterwards with
  `body->setShape(std::make_unique<CircleShape>(r))`
- Polygon shapes (`ShapeType::POLYGON` exists but has no implementation)
- Circles do not appear to rotate, since a uniformly filled circle is
  rotationally symmetric

## Editor setup

`compile_flags.txt` gives clangd the flags it needs. Without it, `.h` files are
parsed as C and C++ standard library headers fail to resolve.
