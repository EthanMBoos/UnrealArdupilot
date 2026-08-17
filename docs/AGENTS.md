# Repository instructions

## Engineering sources of truth

- Read `README.md`, `docs/AERO.md`, and `docs/CPP_STYLE.md` before changing the vehicle model.
- Keep the shared core in SI units with NED world axes and FRD body axes.
- Preserve conventional controls notation, explicit frame/unit suffixes, and the staged equation flow
  documented in `docs/CPP_STYLE.md`.
- Use fixed-size Eigen types in per-tick code. Do not introduce dynamic allocation, I/O, locks,
  randomness, or wall-clock access into the runtime model.

## LLVM tools

- Use the newest stable LLVM toolchain available on the machine. On Apple Silicon macOS, prefer the
  binaries in `/opt/homebrew/opt/llvm/bin` when that directory exists; do not silently fall back to
  an older Apple clang tool when the Homebrew tool is available.
- Use the checked-in `.clang-format`, `.clangd`, and `.clang-tidy` files.
- Generate `build/compile_commands.json` through CMake. Do not hand-edit or commit the generated
  compilation database.
- Use `clangd --check` for compiler-aware diagnostics. Use `clang-tidy` for requested static-analysis
  work, inspect its warnings even when it exits successfully, and never modify fetched third-party
  sources to silence their diagnostics. On macOS, if clang-tidy cannot find standard headers while
  reading an AppleClang compilation database, pass the SDK as `--extra-arg=-isysroot
  --extra-arg=$(xcrun --show-sdk-path)`.

## Python tools

- Use the checked-in `.python-version`, `pyproject.toml`, and `uv.lock`; do not install verification
  dependencies into a global Python interpreter.
- Create or update the local environment with `uv sync --locked` and run scripts with
  `uv run --locked python tools/<script>.py`.
- In a restricted Codex filesystem, point `UV_CACHE_DIR` at a task-specific directory under `/tmp`
  rather than writing to the user's global uv cache.

## Required C++ verification

After changing C++ source, headers, CMake compiler settings, or the C++ tooling configuration:

1. Format only the project C++ files touched by the change with the repository `.clang-format`.
2. Configure with `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   -DUVD_WARNINGS_AS_ERRORS=ON`.
3. Build with `cmake --build build --parallel`.
4. Run `ctest --test-dir build --output-on-failure`.
5. Run `clang-format --dry-run --Werror` on the affected project C++ files.
6. Run `clangd --check=<source> --compile-commands-dir=build --tweaks=` for each affected
   translation unit. Disabling interactive refactoring tweaks keeps check mode focused on parse and
   diagnostics failures. When a header changes, check at least one source or test that directly
   exercises that header.

For aerodynamic, propulsion, rigid-body, integration, trim, or linearization changes, also run the
relevant JSBSim comparison, timestep-convergence, trim, linearization, and deterministic replay
checks described in the repository documentation. Report the exact verification performed and any
check that could not be run.
