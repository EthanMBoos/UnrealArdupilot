# Installation and development setup

This document covers the headless build, Python verification environment, and C++ editor tooling.

## Headless quick start

The P0/B0 and A1-A4 headless fixed-wing path is implemented. From inside the `build/` directory:

```sh
cmake .. && make
```

Then, from the repository root:

```sh
build/uvd validate examples/runs/headless.json
build/uvd simulate examples/runs/headless.json
build/uvd trim examples/runs/headless.json
build/uvd linearize examples/runs/headless.json
```

The simulator writes run bundles under `runs/` unless `--output` selects another empty
directory. `build/uvd replay <run-directory>` reconstructs a simulation from its copied inputs, and
`build/uvd compare <left> <right>` writes JSON metrics, aligned CSV, and an SVG summary.

Create the pinned Python environment and run the reference and timestep checks with
[uv](https://docs.astral.sh/uv/):

```sh
uv sync --locked
uv run --locked python tools/jsbsim_compare.py --samples 200
uv run --locked python tools/timestep_convergence.py
```

The JSBSim fixture is GPL-marked offline material and is excluded from CMake install targets. Unreal,
Cesium, Chaos, ArduPilot JSON/UDP, and closed-loop work still begin at B1/G0/U0.

## C++ editor setup

Use the latest stable LLVM release for `clangd`, `clang-format`, and `clang-tidy`. As of August 2026,
the current stable release is LLVM 22.1.8. The instructions below start from a macOS machine with no
C++ development tools installed.

### macOS from-scratch setup

First install Apple's Command Line Tools. Homebrew LLVM uses the macOS SDK and system C++ library
provided by this package:

```sh
xcode-select --install
```

If `brew` is not installed, install it using the command from the
[official Homebrew site](https://brew.sh/):

```sh
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

At the end, the Homebrew installer prints a `brew shellenv` command. Run that command and add it to
`~/.zprofile` as instructed so `brew` is available in new login shells. On Apple Silicon it normally
looks like this:

```sh
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
```

Install CMake, [uv](https://docs.astral.sh/uv/), and the stable
[Homebrew LLVM package](https://formulae.brew.sh/formula/llvm):

```sh
brew update
brew install cmake llvm uv
```

Homebrew intentionally installs LLVM as a keg-only package, so macOS will otherwise continue to find
Apple's older `/usr/bin/clangd`. Open `~/.zshrc` in an editor:

```sh
nano ~/.zshrc
```

Add these lines at the end, save the file, and start a new terminal or reload it with
`source ~/.zshrc`:

```sh
# Prefer stable Homebrew LLVM over the older Apple command-line tools.
export PATH="$(brew --prefix llvm)/bin:$PATH"
```

Verify both the version and the resolved paths:

```sh
source ~/.zshrc
command -v clangd clang-format clang-tidy
clangd --version
clang-format --version
clang-tidy --version
```

On Apple Silicon, each path should begin with `/opt/homebrew/opt/llvm/bin/`; on Intel macOS it should
normally begin with `/usr/local/opt/llvm/bin/`. A result such as `/usr/bin/clangd` means the `.zshrc`
change has not taken effect. If an editor launched from the Dock does not inherit the shell's `PATH`,
set its clangd executable explicitly to the corresponding Homebrew path.

### Linux setup

Install CMake, a C++ compiler, and Ninja through the distribution package manager. On Debian or
Ubuntu, use the [official LLVM apt repository](https://apt.llvm.org/) to install the current stable
`clangd`, `clang-format`, and `clang-tidy` packages instead of relying on an older distribution
release. Other platforms can use the binaries on the
[LLVM releases page](https://github.com/llvm/llvm-project/releases). Prefer stable releases over
weekly snapshots so contributors format with a released toolchain. Install uv using its
[official installation instructions](https://docs.astral.sh/uv/getting-started/installation/).

## Configure this repository

The Python verification tools use the checked-in `pyproject.toml`, `uv.lock`, and
`.python-version`. Running `uv sync --locked` creates a local `.venv` with the pinned Python and
JSBSim versions; use `uv run --locked python tools/<script>.py` so the scripts never depend on a
global Python installation. The `.venv` directory is generated and ignored by Git.

The repository already contains the shared tooling configuration; users should not recreate these
files locally:

| File | Purpose |
|---|---|
| `.clang-format` | Google-based C++ formatting rules |
| `.clangd` | compilation-database location, indexing, and project diagnostic adjustments |
| `.clang-tidy` | conservative Clang static-analyzer checks limited to project files |
| `AGENTS.md` | required build, test, formatting, and diagnostic checks for Codex sessions |

The quick-start build also writes `build/compile_commands.json`; it is generated build output and is
not committed. Do not create a root-level copy or symlink: the checked-in `.clangd` already points
clangd at `build/` and enables background indexing.

Configure the editor to run the LLVM `clangd` binary. For Visual Studio Code, install the official
`clangd` extension and disable the Microsoft C/C++ language server to avoid duplicate diagnostics.
The [clangd installation guide](https://clangd.llvm.org/installation.html) covers VS Code,
Vim/Neovim, Emacs, Sublime Text, and other LSP-capable editors.

Confirm that clangd can parse each project translation unit with the real CMake flags:

```sh
clangd --check=core/src/fixed_wing.cpp --compile-commands-dir=build --tweaks=
clangd --check=core/src/rigid_body.cpp --compile-commands-dir=build --tweaks=
clangd --check=tools/uvd/aircraft_analysis.cpp --compile-commands-dir=build --tweaks=
clangd --check=tools/uvd/comparison.cpp --compile-commands-dir=build --tweaks=
clangd --check=tools/uvd/configuration.cpp --compile-commands-dir=build --tweaks=
clangd --check=tools/uvd/main.cpp --compile-commands-dir=build --tweaks=
clangd --check=tools/uvd/simulation.cpp --compile-commands-dir=build --tweaks=
clangd --check=tests/test_core.cpp --compile-commands-dir=build --tweaks=
```

The checked-in `.clang-tidy` enables a conservative Clang static-analyzer profile for project files.
On macOS, run it against the compilation database with the active SDK path:

```sh
clang-tidy -p build core/src/fixed_wing.cpp \
  --extra-arg=-isysroot --extra-arg="$(xcrun --show-sdk-path)"
```

The checked-in `.clang-format` applies the repository's Google-based formatting rules. From the
repository root, format or verify all current C++ files with:

```sh
clang-format -i core/include/uvd/*.hpp core/src/*.cpp tools/uvd/*.cpp tests/*.cpp
clang-format --dry-run --Werror \
  core/include/uvd/*.hpp core/src/*.cpp tools/uvd/*.cpp tests/*.cpp
```
