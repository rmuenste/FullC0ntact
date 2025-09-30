# Repository Guidelines

## Project Structure & Key Modules
Core dynamics live in `inshape3dcore/` (`world.h` orchestrates simulation state, `rigidbody.h` handles kinematics). Contact solvers and numerical kernels sit in `math/` (e.g., `lcpsolvergaussseidel.h`), utilities under `util/`, and GPU extensions in `cuda_addon/`. Third-party helpers either ship in `libs/` (rapidxml, Eigen) or are fetched on demand (OpenMesh/OpenVolumeMesh). Scenario drivers live in `applications/`; meshes and inputs under `meshes/` and `path/`. Keep build artefacts out. Tests stay in `unit_tests/` behind CMake options.

## Build, Test, and Development Commands
Work out of tree:
- `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_FC_UNIT_TESTS=ON` — configure with unit tests enabled.
- `cmake --build build --target all -j` — compile the core library plus default applications.
- `ctest --test-dir build --output-on-failure` — run registered CTest targets.
Pass feature flags (e.g., `-DUSE_OPENMESH=ON`, `-DFC_CUDA_SUPPORT=ON`) when you need the feature; CMake fetches missing mesh libraries automatically, while CUDA still assumes a local toolkit.

## Coding Style & ABI Notes
Follow Allman braces with two-space indentation and trail member fields with `_` (`velocity_`). Use CamelCase for types, lowercase descriptive filenames, and prefer `//` comments. Preserve the C ABI exposed to Fortran callers: avoid breaking struct layouts or exported function signatures; extend behaviour via new optional parameters or entry points. Guard logging and diagnostics behind preprocessor switches.

## Testing Guidelines
Each solver or subsystem has a matching directory in `unit_tests/`. Register new coverage with `ADD_TEST` in the local CMakeLists, respecting feature flags. Name executables after the feature (`integration`, `json_reader`) and run `ctest` before submitting changes, noting any skipped cases in the review.

## Commit & Pull Request Guidelines
Write short, imperative commit titles without trailing periods (`Handle CMake warnings`). Scope commits narrowly, documenting ABI or solver impacts in the body. Pull requests should summarise motivation, list toggled CMake options from `cmake_scripts/ConfigureAsProject.cmake`, include local test output, and attach artefacts (logs, screenshots) for visual demos. Link upstream issue IDs and flag effects on Fortran integrations.

## Configuration Flags & Dependencies
Common options live in `cmake_scripts/ConfigureAsProject.cmake`; give new toggles sensible defaults and document required libraries. Optional dependencies should use `FetchContent` (see the OpenMesh/OpenVolumeMesh setup) so they download only when enabled and remain guardable via `option(...)` / `cmake_dependent_option`.

## Modernization Roadmap
Modernise in reviewed iterations: choose a subsystem, tighten tests, land the cleanup, then advance. First milestone targets the CMake toolchain (`CMakeLists.txt`, `cmake_scripts/`) to adopt modern targets, interface usage requirements, and consistent option naming before touching runtime code.
