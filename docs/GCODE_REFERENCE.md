# G-code and M-code Reference

The codes BreakoutMill and BreakoutCut accept, in loaded programs and in MDI. The parser is in
[`gcode_parser.cpp`](../src/core/gcode/gcode_parser.cpp) and the MDI handling is in
[`machine_controller.cpp`](../src/core/controller/machine_controller.cpp).

A program with errors won't start: Cycle Start is refused until the errors are fixed. Warnings don't stop a program.
Right after loading, the program panel lists every error and warning with its line, together with the checks against
the machine that Cycle Start makes (soft limits, homing, tool table) and a few machine warnings, such as a feed above
the motor settings. The checks are in [`program_check.cpp`](../src/core/controller/program_check.cpp).

## G codes

| Code | Function | Notes |
|---|---|---|
| G0 / G1 | Rapid / linear feed | G1 needs a feed rate (F) |
| G2 / G3 | CW / CCW arc | I/J/K or R; helical moves and full circles work (full circles need I/J/K) |
| G4 | Dwell | Needs P; seconds by default, or milliseconds with the Mach3 option |
| G17 / G18 / G19 | XY / XZ / YZ plane | |
| G20 / G21 | Inches / mm | G70 / G71 do the same |
| G28 | Return to machine zero | Optional intermediate point; with no axis words, X, Y and Z go home |
| G30 | Same as G28 | Gives a warning, because no secondary position is stored |
| G43 H\<n\> | Tool length offset | Without H, uses the current tool; only applied in tool table mode |
| G49 | Cancel tool length offset | |
| G53 | Move in machine coordinates | Applies to that block only; needs G0 or G1 mode and G90 |
| G54–G59 | Work offsets | |
| G61 / G61.1 | Exact stop | |
| G64 [P\<tol\>] | Constant velocity | P sets corner tolerance, limited to 0.001–5 mm |
| G68 A\<x\> B\<y\> R\<deg\> [I] | Rotate the program (Mach3 form) | See [G68 rotation](#g68-rotation) |
| G69 | Cancel G68 rotation | |
| G73 / G81 / G82 / G83 | Drilling cycles | High-speed peck, simple, dwell, deep peck; G90 only |
| G80 | Cancel drilling cycle | |
| G98 / G99 | Cycle return to initial Z / R plane | |
| G90 / G91 | Absolute / incremental XYZ | |
| G90.1 / G91.1 | Absolute / incremental arc centres | G91.1 is the default |
| G92 | Set coordinate offset | Needs at least one axis word |
| G92.1 / G92.2 | Clear G92 offsets | Both behave the same; G92.2 doesn't suspend the offsets for later restore |

**Accepted but do nothing:** G15, G40, G50 and G94. These are the "off" or default states, so they are safe in
CAM preambles.

### G68 rotation

`G68 A<x> B<y> R<degrees>` turns every following X/Y point of the program about the centre (A, B) by R degrees,
counterclockwise for a positive R, until `G69`. It works as in Mach3:

- A and B are the X and Y of the centre (on a G68 line, A is not the A axis). R is the angle in degrees.
- A word that is left out keeps its last value (at first 0, so `G68 R30` turns about the work zero).
- With an I word (its value is ignored), the given A, B and R are added to the current rotation instead of replacing
  it: `G68 I0 R15` turns 15 degrees further.
- Incremental moves (G91), arc centres (I/J, also with G90.1) and drilling cycle positions turn with the program.
  Z, the A axis and G53 / G28 machine moves are not rotated.

These give an error: X, Y, Z, J or K on the G68 line itself, and G92 or arcs in G18 / G19 while a rotation is active.
G68 is refused in MDI, because each MDI command is run on its own.

**MDI only:** G10 L2 or L20 P1–P6 sets work offsets. In a program, it's an "Unsupported G-code" error.

**MDI difference:** G92 in MDI permanently changes the active work offset, while in a program it's only a shift inside
that program.

**Rejected with an error:**

- G41 / G42 (cutter compensation)
- G44, G43.1 and G43.2
- G93 / G95
- Any other G code, including G31 and G38.x; probing is done from the Probing panel and Lua scripts instead

## M codes

| Code | Function | Notes |
|---|---|---|
| M0 / M1 | Program stop | M1 always stops; there's no optional-stop switch |
| M2 / M30 | Program end | Spindle and coolant go off when the program ends. Later blocks are ignored, with a warning |
| M3 / M4 | Spindle or torch on, CW / CCW | M4 gives dynamic power on a laser |
| M5 | Spindle or torch off | |
| M6 | Tool change | Uses the T word; behaviour depends on the M6 mode setting |
| M7 / M8 | Mist / flood coolant on | |
| M9 | Coolant off | |
| M48 / M49 | Override enable / disable | Accepted but have no effect |
| M100–M999 | Lua macros | **MDI only**, runs `scripts/macros/M<n>.lua` |

**Rejected with an error:** M98 / M99 (subprograms).

**Ignored with a warning:** any other M code in a program. That includes M100–M999, so a macro call in a loaded program
doesn't run.

## Other words

- **Accepted:** F, S and T (T1–T255), plus X, Y, Z, A, I, J, K, R, P and Q.
- **Parsed but ignored:** N, O, H, D and L, except that G43 uses H.
- **Rejected:** `#` parameters and `[ ]` expressions.
