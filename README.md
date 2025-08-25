[![Releases](https://img.shields.io/github/v/release/amitkumarghb/bpt_veach.svg)](https://github.com/amitkumarghb/bpt_veach/releases)

# Bi-Directional Path Tracer with MIS | BPT_Veach Renderer

![BPT render example](https://upload.wikimedia.org/wikipedia/commons/5/54/Global_illumination_rendering.jpg)

Tags: bi-directional, bidirectional, global-illumination, path-tracer, path-tracing, pathtracer, pathtracing, ray-tracer, ray-tracing, raytracer, raytracing

A compact, modern implementation of a bi-directional path tracer (BDPT) with multiple importance sampling (MIS). The project targets research, learning, and production testing for unbiased global illumination. It supports common geometry, materials, and camera models. Use the Releases page to get a runnable binary. The release file must be downloaded and executed.

- Releases: https://github.com/amitkumarghb/bpt_veach/releases

---

## Key features 🎯

- Bi-directional path tracing (Veach-style BDPT).
- Multiple importance sampling (balance heuristic).
- Support for diffuse, glossy, and dielectric materials.
- Importance sampling of area lights and environment maps.
- Subpath connection strategies with path reuse.
- Russian roulette and adaptive path length.
- HDR output (OpenEXR / PNG with tone mapping).
- Small C++ codebase with clear module breakdown.
- Example scenes and reference renders.

---

## When to use this renderer

- Research on light transport and sampling.
- Baseline for unbiased algorithms and MIS experiments.
- Teaching path tracing concepts in classes.
- Generating reference renders for scenes with complex indirect light.

---

## Quick start — download and run ⚡

1. Visit the Releases page and download the appropriate release asset for your OS. The release file must be downloaded and executed.
   - Releases: https://github.com/amitkumarghb/bpt_veach/releases

2. Example run on Linux/macOS (after download):
   ```bash
   chmod +x bpt_veach-linux
   ./bpt_veach-linux --scene scenes/cornell_box.json --spp 1024 --out output.exr
   ```

3. Example run on Windows (PowerShell):
   ```powershell
   .\bpt_veach-win.exe --scene scenes/cornell_box.json --spp 1024 --out output.exr
   ```

The release binary includes example scenes in a `scenes/` folder. The tool writes HDR output by default. Use an EXR viewer or convert to PNG with tonemapping.

---

## Build from source 🛠️

The project uses CMake. The build targets Linux, macOS, and Windows. The code relies on a few common libraries. Below are typical steps and a suggested dependency list.

Recommended dependencies
- CMake >= 3.12
- A modern C++ compiler (GCC, Clang, MSVC) with C++17 support
- Eigen (math)
- TinyEXR or OpenEXR
- stb_image / stb_image_write
- GLFW (optional for interactive viewer)
- Embree (optional for faster ray intersection)
- OpenMP (optional for multithreading)

Typical build steps:
```bash
git clone https://github.com/amitkumarghb/bpt_veach.git
cd bpt_veach
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

Common CMake knobs:
- -DUSE_EMBREE=ON  # enable Embree for faster ray queries
- -DUSE_OPENEXR=ON # enable OpenEXR support
- -DBUILD_VIEWER=ON # build a simple GL viewer

Build on Windows:
- Open an x64 Native Tools shell.
- Run the same CMake steps and build with MSBuild or Visual Studio.

---

## Usage and CLI reference

Core command-line options:
```
--scene <path>     Path to scene JSON file or .obj
--out <path>       Output filename (.exr recommended)
--spp <int>        Samples per pixel (total)
--max-depth <int>  Maximum path depth (default 8)
--threads <int>    Number of worker threads
--seed <int>       RNG seed
--preview          Low-res live preview (if viewer built)
--env <path>       HDR environment map
--denoise <bool>   Apply denoiser after render
```

Example:
```bash
./bpt_veach --scene scenes/kitchen.json --spp 4096 --max-depth 12 --out kitchen.exr
```

Scene files:
- Scenes use JSON. They define geometry, materials, lights, camera, and integrator parameters.
- The `scenes/` folder includes examples: `cornell_box.json`, `living_room.json`, `glass_test.json`.

---

## Scene format (short)

A scene JSON includes:
- camera: type, position, lookat, fov
- materials: name, type (diffuse, glossy, glass), params
- meshes: file, material, transform
- lights: emissive meshes or area lights
- integrator: type (bdpt), maxDepth, rrStart, connectStrategy

Example snippet:
```json
{
  "camera": {
    "type": "perspective",
    "pos": [0,1,3],
    "lookat": [0,1,0],
    "fov": 45
  },
  "integrator": {
    "type": "bdpt",
    "maxDepth": 10,
    "rrStart": 5
  }
}
```

---

## Algorithm notes — how BDPT works (brief)

- The renderer builds two subpaths per sample: one from the camera and one from a light source.
- It attempts to connect vertices from both subpaths to form complete light transport paths.
- It computes a weight for each connection using multiple importance sampling (MIS) with the balance heuristic.
- The implementation uses probability densities from sampling functions to compute relative weights.
- BDPT reduces variance for scenes with difficult light paths, such as strong caustics or small area lights.

Key jargon: subpath, connection, MIS, balance heuristic, Russian roulette, contribution, throughput, PDF.

---

## Output formats and post-processing

- Default output: OpenEXR (.exr). Use a viewer or convert to PNG for quick previews.
- Tone mapping: ACES and Reinhard options.
- Denoiser: Optional pointer to a fast denoiser (e.g., Intel Open Image Denoise). Enable via `--denoise true`.

Convert EXR to PNG (example using ImageMagick):
```bash
convert output.exr -auto-level -colorspace sRGB output.png
```

---

## Performance tips

- Use Embree for faster ray-scene queries on CPU.
- Lower maxDepth for scenes with mostly diffuse light.
- Use stratified or low-discrepancy sampling for camera and light samples.
- Increase rrStart to cut small contributions early.
- Use HDR environment maps with importance sampling to reduce variance.

---

## Example renders and gallery 🖼️

![Cornell Box](https://upload.wikimedia.org/wikipedia/commons/6/6f/Cornell_box.png)
*Cornell Box with soft area light and global illumination.*

Public repositories and tutorials often include test scenes you can adapt. The `scenes/` folder in this repo shows how to set up area lights, emissive textures, and glass materials.

---

## Tests and validation ✅

- Unit tests cover sampling routines and PDF calculations.
- Reference scenes compare outputs to path-tracing ground truth.
- Use the `--seed` option to reproduce renders.

Run the test suite:
```bash
cmake -DBUILD_TESTS=ON ..
ctest --output-on-failure
```

---

## Contributing

- Fork the repo and make a branch per feature or bugfix.
- Follow the existing code style and keep modules small.
- Add tests for new samplers and integrator changes.
- Document any algorithm changes in the `docs/` folder.

Pull requests should include:
- A short description of the change.
- Repro steps or a test scene.
- Performance notes if the change affects speed.

---

## FAQ

Q: Where do I get the executable?
A: Download the release asset from the Releases page. The release file must be downloaded and executed.

Q: Does this support GPU acceleration?
A: The current mainline build targets CPU. GPU support is experimental in a feature branch.

Q: Can I use my own denoiser?
A: Yes. The renderer can output auxiliary buffers (albedo, normal) for external denoisers.

---

## Licensing

The project uses an open-source license in LICENSE file. Check the repository for exact terms.

---

## Links and resources

- Releases and download: https://github.com/amitkumarghb/bpt_veach/releases
- Sample scenes: see `scenes/` in the repo
- Papers:
  - Veach, Eric. "Robust Monte Carlo Methods for Light Transport Simulation."
  - Veach and Guibas. "Bidirectional estimators for light transport."

---

## Contact and maintainers

- Repository: amitkumarghb/bpt_veach
- Open issues on GitHub for bug reports and feature requests.

