# Third-Party Software

BreakoutCNC does not include third-party source code in this repository. CMake downloads the following libraries
at configure time and links them into the executables. Their licenses must be shipped with any binary release.

| Library | Version | License | Source |
|---|---|---|---|
| Dear ImGui (docking branch) | 1.93.1 WIP, commit `d88f238` | MIT | https://github.com/ocornut/imgui |
| GLFW | 3.4 | zlib/libpng | https://github.com/glfw/glfw |
| Lua | 5.4.7 | MIT | https://github.com/lua/lua |
| hidapi (Linux / macOS builds only) | 0.14.0 | BSD-3-Clause (one of its license options) | https://github.com/libusb/hidapi |

## Trademarks and interoperability

RNR, RNR Eco Motion, Mach3, Huanyang and other product names belong to their owners. BreakoutCNC is an independent
project and is not affiliated with or endorsed by any of them. The RNR USB protocol and the Huanyang VFD plugin
settings were documented for interoperability (see [docs/protocol](docs/protocol)); no vendor code or binaries are
included.
