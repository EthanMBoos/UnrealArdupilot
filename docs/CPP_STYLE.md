# Controls-oriented C++ style

The [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) is the repository
baseline. Run clang-format with the checked-in `.clang-format`, which selects its Google preset.
The core is engineering software first, so the additions and exceptions below make the governing
equations, frames, and units easy to audit against the model documentation.

## Project adaptations

- The shared core targets C++23 because CMake and Unreal compile the same sources with the required
  language features.
- Existing `.cpp` and `.hpp` extensions are retained to match the CMake and Unreal project layout.
- Configuration and CLI code may throw exceptions. Per-tick model and integration code remains
  `noexcept`.
- Functions and methods use `snake_case()`. Types use `PascalCase`; constants retain the Google
  `kConstantName` convention.
- Conventional controls notation and explicit frame/unit suffixes take precedence over ordinary
  identifier casing when they make an equation recognizable.
- Eigen is the approved fixed-size runtime linear-algebra library. Its standard operators are
  allowed where they express the written mathematics directly.

## Write equations in stages

Keep each pure function at one mathematical level:

```text
state -> air data -> nondimensional coefficients -> dimensional wrench -> state derivative
```

Do not combine coefficient construction, dimensionalization, frame conversion, and integration in
one function. Keep expressions in the same order as their equations in [AERO.md](AERO.md).

## Linear algebra

Use the fixed-size aliases from `uvd/core.hpp` for runtime math:

```cpp
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using Matrix3 = Eigen::Matrix3d;
using Quaternion = Eigen::Quaterniond;
```

Use Eigen operations such as `.dot()`, `.cross()`, `.norm()`, `.transpose()`, and matrix-vector
multiplication instead of component-wise reimplementations. Dynamic Eigen matrices belong in
offline tools such as trim and linearization, not in the per-tick vehicle model.

Quaternion construction is `(w, x, y, z)`, but Eigen's raw `coeffs()` order is `(x, y, z, w)`.
Conversions that expose coefficients must name their ordering and have a convention test.

## Names carry engineering meaning

- Put the frame before the unit: `force_body_N`, `wind_ned_mps`, `omega_body_radps`.
- Use uppercase SI symbols where lowercase would be ambiguous. In particular, `_N` means newtons;
  `_ned` or `_body` identifies a frame.
- Use conventional coefficient names inside equation code: `C_L`, `C_D`, `C_Y`, `C_ell`, `C_m`,
  and `C_n`.
- Prefer named domain structs over vector indices for physically different values.
- Short mathematical names such as `C`, `d`, `qbar_S_N`, and `R_body_to_ned` are appropriate within
  a small equation-focused scope. Public interfaces use descriptive names.
- At an equation boundary, bind descriptive state members to the symbols used in the written model,
  such as `alpha`, `beta`, `delta_e`, and `V_a`. This keeps the equation visually comparable to its
  reference without leaking abbreviated names into the public interface.
- Prefer designated initializers for domain structs so call sites do not depend on member order.
- Give polynomial terms names such as `x2`, `x1`, and `x0`; do not encode a coefficient-order
  convention in bare array indices.

## Runtime boundary

Per-tick model functions are deterministic, `noexcept`, and free of I/O, locks, random calls, wall
clock access, and dynamic-size linear algebra. Configuration and CLI code may allocate and throw.
Unreal types stop in the Unreal component; the core uses SI and NED/FRD throughout.

## Formatting and verification

Format C++ with the repository `.clang-format`; do not hand-format around it. New equation stages
require focused unit tests plus the existing JSBSim, trim, linearization, timestep-convergence, and
replay checks as appropriate.
