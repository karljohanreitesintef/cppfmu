# Plan: Ease-of-use & Documentation Improvements

## Context

CPPFMU is a header + `.cpp` helper library for writing FMI co-simulation
slaves in C++ (FMI 1.0/2.0 stable, FMI 3.0 partial). The code and inline
header docs are solid, but a newcomer hits several friction points going from
"clone the repo" to "a working `.fmu`". This is **documentation + examples +
doc-correctness fixes**, not a functional rewrite of the library.

**Scope (confirmed with user): Comprehensive.**
- Fix README inaccuracies.
- Add worked **mass-spring-damper** examples for **both FMI 2.0 and FMI 3.0**.
- Add a `CHANGELOG.md`.
- Split docs into a `docs/` tree if the README grows too large.
- Build/consumption docs focus on the **Conan/CMake package flow**.

## Findings (friction points discovered)

### Documentation bugs / inaccuracies (`README.md`)
1. **Broken Conan `generate()` snippet.** `if dep.ref.name == "cppfmu":` is
   dedented *outside* the `for` loop, so the copy never runs as written. The
   correct, working pattern already exists in `test_package/conanfile.py`
   (which also copies `fmi3_functions.cpp`).
2. **Typo `fmi_function.cpp`** (missing `s`) in the same snippet — real file
   is `fmi_functions.cpp`.
3. **CMake package/options undocumented.** `CMakeLists.txt` exposes
   `CPPFMU_FMI_1`, `CPPFMU_FMI_3`, `CPPFMU_FMI_ALL` and installs a
   `cppfmu::cppfmu` target — none mentioned in the README.
4. **FMI 3.0 `CppfmuInstantiateSlave` signature** is only "see the header".
   The 11-parameter form (from `cppfmu_cs_fmi3.hpp`) should be shown inline.
5. **`use_fmi_version` option undocumented.** The Conan recipe supports
   `1 | 2 | 3 | "all"`; the README only shows version `3`.

### Missing material
6. **No `examples/` directory.** The only usage samples are the test slaves
   in `tests/`, which are contrived for coverage (magic value references,
   derivative toggles, binary types) — not a discoverable "how do I start".
7. **No `modelDescription.xml` anywhere.** Mapping value references to
   `Get/SetReal` indices is exactly where beginners struggle; no sample given.
8. **No `.fmu` packaging guidance** (ZIP layout: `modelDescription.xml` +
   `binaries/<platform>/<name>.{so,dll,dylib}` + `resources/`).
9. **No CHANGELOG** despite `version.txt` = 1.3.0 and active FMI 3.0 work.

## Approach

### A. Fix & extend README (Conan/CMake focus)
- Fix the `generate()` dedent bug and `fmi_function.cpp` typo by aligning the
  snippet with `test_package/conanfile.py` (known-good).
- Document the Conan `use_fmi_version` option values (`1 | 2 | 3 | all`).
- Add a **"Consuming via CMake"** subsection using the `cppfmu::cppfmu`
  target and the `${cppfmu_pkg}/src/fmi*_functions.cpp` copy pattern from
  `test_package/CMakeLists.txt`.
- Add the FMI 3.0 `CppfmuInstantiateSlave` signature inline.
- Add a **"Packaging a `.fmu`"** subsection (ZIP layout + platform dir names).
- Link out to the new `examples/` from the Usage section.

### B. Add `examples/` — mass-spring-damper (FMI 2.0 + FMI 3.0)
A physically meaningful model that exercises the parts beginners actually
need: **parameters** (mass, stiffness, damping), **state** (position,
velocity), **input** (external force), **output** (position, velocity),
and real **`DoStep` integration** (semi-implicit Euler).

Layout:
```
examples/
  README.md                     # what the model is, VR table, build, packaging
  mass_spring_damper/
    model.hpp                   # shared physics (ODE + integrator), FMI-agnostic
    fmi2/
      mass_spring_damper.cpp    # SlaveInstance + CppfmuInstantiateSlave
      modelDescription.xml      # FMI 2.0 schema, VRs matching the .cpp
      CMakeLists.txt            # builds the shared-lib module via cppfmu::cppfmu
      conanfile.py             # requires cppfmu with use_fmi_version=2
    fmi3/
      mass_spring_damper.cpp    # SlaveInstance3 + FMI 3.0 CppfmuInstantiateSlave
      modelDescription.xml      # FMI 3.0 schema, Float64 VRs
      CMakeLists.txt
      conanfile.py             # requires cppfmu with use_fmi_version=3
```
- Keep the **physics in `model.hpp`** so the FMI 2.0 and FMI 3.0 slaves are
  thin adapters — this doubles as a "separate your model from the FMI glue"
  teaching point and keeps the two examples consistent.
- Value-reference constants named in code (e.g. `VR_MASS`, `VR_FORCE`,
  `VR_POSITION`) and matched exactly in `modelDescription.xml`, with a table
  in `examples/README.md`.
- FMI 2.0 slave uses `cppfmu::AllocateUnique` + `cppfmu::Memory`; FMI 3.0
  slave uses `cppfmu::AllocateUnique3` + `std::function` logger — mirroring
  `tests/cs_slave.cpp` and `tests/cs_slave_fmi3.cpp` respectively.

### C. Add `CHANGELOG.md`
Reconstruct notable entries from `git log` (FMI 3.0 support, combined
FMI 2.0+3.0 build, lifecycle extraction, dispatch centralization), anchored
to `version.txt` = 1.3.0. Keep-a-Changelog format.

### D. Optional docs/ split
Only if the README becomes unwieldy after A: move Memory/Logging/Error
deep-dives into `docs/` and keep the README a lean quickstart + index. Decide
during execution; not committed up front.

## Files to modify / create
- `README.md` — fixes + Conan option docs + CMake consume section + FMI 3.0
  signature + `.fmu` packaging + link to examples.
- `examples/README.md` — new.
- `examples/mass_spring_damper/model.hpp` — new (shared physics).
- `examples/mass_spring_damper/fmi2/{mass_spring_damper.cpp,modelDescription.xml,CMakeLists.txt,conanfile.py}` — new.
- `examples/mass_spring_damper/fmi3/{mass_spring_damper.cpp,modelDescription.xml,CMakeLists.txt,conanfile.py}` — new.
- `CHANGELOG.md` — new.
- (Optional) `docs/*.md` — only if README split is warranted.

## Reuse (existing code the examples/docs build on)
- `tests/cs_slave.cpp` — FMI 2.0 `SlaveInstance` + `CppfmuInstantiateSlave` pattern (memory, `AllocateUnique`, FMU-state).
- `tests/cs_slave_fmi3.cpp` — FMI 3.0 `SlaveInstance3` + `AllocateUnique3` + enhanced `DoStep` output params.
- `cppfmu_cs_fmi3.hpp` — exact FMI 3.0 instantiation signature to quote in README.
- `test_package/conanfile.py` — the **correct** `fmi*_functions.cpp` copy pattern (fixes README bug #1/#2).
- `test_package/CMakeLists.txt` — reference for consuming the package + locating `${cppfmu_pkg}/src/fmi*_functions.cpp`.
- `conanfile.py` — `use_fmi_version` option values to document.

## Steps
- [x] Fix README Conan `generate()` dedent bug + `fmi_function.cpp` typo (align to `test_package/conanfile.py`).
- [x] Document Conan `use_fmi_version` values (`1 | 2 | 3 | all`) in README.
- [x] Add README "Consuming via CMake" subsection (`cppfmu::cppfmu` + fmi*_functions.cpp copy).
- [x] Add README FMI 3.0 `CppfmuInstantiateSlave` signature snippet.
- [x] Add README "Packaging a `.fmu`" subsection (ZIP layout + platform dirs).
- [x] Create `examples/mass_spring_damper/model.hpp` (shared physics/integrator).
- [x] Create FMI 2.0 example: slave `.cpp`, `modelDescription.xml`, `CMakeLists.txt`, `conanfile.py`.
- [x] Create FMI 3.0 example: slave `.cpp`, `modelDescription.xml`, `CMakeLists.txt`, `conanfile.py`.
- [x] Write `examples/README.md` (model description, VR table, build + packaging steps).
- [x] Link `examples/` from the README Usage section.
- [x] Add `CHANGELOG.md` (Keep-a-Changelog, reconstruct from git log, anchor 1.3.0).
- [ ] (Not done -- README stayed manageable) Split deep-dive docs into `docs/` if README grows too large.

## Verification
- README code blocks diffed against `test_package/conanfile.py` /
  `test_package/CMakeLists.txt` (the known-good consumption patterns).
- Each example builds as a shared-library module through its own Conan +
  CMake flow using `cppfmu::cppfmu` (documented; CI wiring optional).
- `modelDescription.xml` files validated (xmllint against FMI 2.0 / 3.0
  schemas, or manual review) and their value references cross-checked against
  the `VR_*` constants in the corresponding `.cpp`.
- Physics sanity check: with zero force the mass-spring-damper decays to rest;
  spot-check a few `DoStep` outputs.
- `CHANGELOG.md` entries cross-checked against `git log`.
```
