// Built-in sample programs (shown at startup and from the demo buttons)
#include "core/gcode/gcode_parser.h"
#include <cmath>
#include <sstream>
#include <string>

namespace rnr::gcode {

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

std::string GCodeParser::samplePlasmaBracketGCode() {
    return R"(; =========================================================
; BreakoutCut Demo Part: Heavy-Duty Gusset Bracket (140x90mm)
; Features: 2x Bolt Holes (CCW Inside Cut) + Outer Contour
; Units: mm (G21) | Coordinate Mode: Absolute (G90)
; =========================================================
G21 G90 G91.1
F1800

; --- Hole 1: 24mm Diameter at (X35, Y45) ---
G0 Z10.0
G0 X35.0 Y45.0        ; Rapid to Hole 1 center (lead-in start)
G0 Z3.8               ; Pierce Height
M3                    ; Fire Plasma Torch ON (OUT1)
G4 P0.5               ; Pierce Delay 0.5s
G1 Z1.5 F900          ; Drop to Cut Height (1.5mm)
G1 X47.0 Y45.0 F1500  ; Lead-in to circle edge (R=12mm)
G3 X47.0 Y45.0 I-12.0 J0.0 F1600 ; Full CCW Inside Circle Cut
M5                    ; Torch OFF
G0 Z10.0              ; Retract to Safe Z

; --- Hole 2: 20mm Diameter at (X100, Y35) ---
G0 X100.0 Y35.0       ; Rapid to Hole 2 center (lead-in start)
G0 Z3.8               ; Pierce Height
M3                    ; Fire Plasma Torch ON (OUT1)
G4 P0.5               ; Pierce Delay 0.5s
G1 Z1.5 F900          ; Drop to Cut Height
G1 X110.0 Y35.0 F1500 ; Lead-in to circle edge (R=10mm)
G3 X110.0 Y35.0 I-10.0 J0.0 F1600 ; Full CCW Inside Circle Cut
M5                    ; Torch OFF
G0 Z10.0              ; Retract to Safe Z

; --- Outer Gusset Profile (CW Outside Cut) ---
G0 X-5.0 Y10.0        ; Rapid to exterior lead-in
G0 Z3.8               ; Pierce Height
M3                    ; Fire Plasma Torch ON (OUT1)
G4 P0.6               ; Pierce Delay 0.6s
G1 Z1.5 F900          ; Drop to Cut Height
G1 X0.0 Y10.0 F1800   ; Lead-in onto left edge
G1 X0.0 Y80.0         ; Left vertical edge
G2 X10.0 Y90.0 I10.0 J0.0 ; Top-left 10mm corner radius
G1 X75.0 Y90.0        ; Top horizontal edge
G1 X140.0 Y55.0       ; Angled gusset slope
G1 X140.0 Y10.0       ; Right vertical edge
G2 X130.0 Y0.0 I-10.0 J0.0 ; Bottom-right 10mm corner radius
G1 X10.0 Y0.0         ; Bottom horizontal edge
G2 X0.0 Y10.0 I0.0 J10.0  ; Bottom-left 10mm corner radius
M5                    ; Torch OFF
G0 Z15.0              ; Safe Z Park
G0 X0.0 Y0.0          ; Return to Work Origin
M30                   ; End of Program
)";
}

std::string GCodeParser::sampleLaserCoasterGCode() {
    // Generated rather than written out: the rose curve, raster fill and tick marks are ~1200 moves
    constexpr double cx = 45.0, cy = 45.0;
    std::ostringstream g;
    g.setf(std::ios::fixed);
    g.precision(3);
    auto polar = [&](double r, double a, double& x, double& y) {
        x = cx + r * std::cos(a);
        y = cy + r * std::sin(a);
    };

    g << "; =========================================================\n"
         "; BreakoutCut Laser Demo: Compass Rose Coaster (90x90mm, 3mm plywood)\n"
         "; Engrave: raster-filled center, 8-petal rose, rings, 72 compass ticks\n"
         "; Cut: rounded outline at full laser power\n"
         "; M4 = dynamic laser power (scaled by speed), S0-1000 = 0-100% power\n"
         "; =========================================================\n"
         "G21 G90 G17\n";

    // Raster fill of the center disc (bidirectional scan lines, 0.25 mm line spacing)
    g << "\n; --- Raster engrave: center disc R7 (laser power 45%) ---\n";
    constexpr double kDiscR = 7.0, kLineStep = 0.25;
    bool first_line = true, left_to_right = true;
    for (double dy = -kDiscR + kLineStep * 0.5; dy < kDiscR; dy += kLineStep) {
        const double hw = std::sqrt(kDiscR * kDiscR - dy * dy);
        const double x0 = left_to_right ? cx - hw : cx + hw;
        const double x1 = left_to_right ? cx + hw : cx - hw;
        g << "G0 X" << x0 << " Y" << cy + dy << "\n";
        if (first_line) { g << "M4 S450\n"; first_line = false; }
        g << "G1 X" << x1 << " F4000\n";
        left_to_right = !left_to_right;
    }
    g << "M5\n";

    // Rose curve r = R * cos(4t): 8 petals, one continuous line
    g << "\n; --- Vector engrave: 8-petal rose R28 (laser power 35%) ---\n";
    constexpr double kRoseR = 28.0;
    constexpr int kRoseSteps = 960;
    for (int i = 0; i <= kRoseSteps; ++i) {
        const double t = 2.0 * kPi * i / kRoseSteps;
        double x, y;
        polar(kRoseR * std::cos(4.0 * t), t, x, y);
        if (i == 0) g << "G0 X" << x << " Y" << y << "\nM4 S350\n";
        else        g << "G1 X" << x << " Y" << y << (i == 1 ? " F2400" : "") << "\n";
    }
    g << "M5\n";

    // Two rings (full arcs)
    g << "\n; --- Vector engrave: rings R32 and R34 (laser power 35%) ---\n";
    for (double r : {32.0, 34.0}) {
        g << "G0 X" << cx + r << " Y" << cy << "\nM4 S350\n"
          << "G3 X" << cx + r << " Y" << cy << " I" << -r << " J0.000 F2400\nM5\n";
    }

    // Compass ticks every 5 degrees, longer at 45 / 90 degrees
    g << "\n; --- Vector engrave: 72 compass ticks (laser power 40%) ---\n";
    for (int i = 0; i < 72; ++i) {
        const double a = 2.0 * kPi * i / 72.0;
        const double r_out = (i % 18 == 0) ? 41.0 : (i % 9 == 0) ? 39.5 : (i % 2 == 0) ? 38.0 : 37.0;
        double x0, y0, x1, y1;
        polar(35.5, a, x0, y0);
        polar(r_out, a, x1, y1);
        g << "G0 X" << x0 << " Y" << y0 << "\nM4 S400\nG1 X" << x1 << " Y" << y1 << " F2000\nM5\n";
    }

    // Outline cut: 90x90 square with R8 corners, constant power (M3), 2 passes
    g << "\n; --- Cut: 90x90 outline, R8 corners, 2 passes (laser power 100%, air assist) ---\n"
         "G0 X0.000 Y8.000\n"
         "M3 S1000\n";
    for (int pass = 0; pass < 2; ++pass) {
        g << "G1 X0.000 Y82.000 F600\n"
             "G2 X8.000 Y90.000 I8.000 J0.000\n"
             "G1 X82.000 Y90.000\n"
             "G2 X90.000 Y82.000 I0.000 J-8.000\n"
             "G1 X90.000 Y8.000\n"
             "G2 X82.000 Y0.000 I-8.000 J0.000\n"
             "G1 X8.000 Y0.000\n"
             "G2 X0.000 Y8.000 I0.000 J8.000\n";
    }
    g << "M5\n"
         "G0 X0.000 Y0.000\n"
         "M30\n";
    return g.str();
}

std::string GCodeParser::sampleMillAtcGCode() {
    return R"((=========================================================)
(BreakoutMill Mach3 ATC Demo: Bearing Housing Block 120x80mm   )
(T1 D=6.0mm Flat Endmill - Circular Pocket & Outer Contour)
(T2 D=4.0mm Spot/Twist Drill - 4x Corner Bolt Holes G83   )
(=========================================================)
G90 G94 G91.1 G40 G49 G17
G21
G28 G91 Z0.
G90

(--- OPERATION 1: 6mm Flat Endmill Pocket & Profile ---)
T1 M06 (T1 D=6.0mm 4-Flute Carbide Flat Endmill)
G43 H1 Z15.0
S14000 M03
G0 X60.0 Y40.0
G0 Z3.0
G1 Z-3.0 F400
G1 X75.0 Y40.0 F1200
G3 X75.0 Y40.0 I-15.0 J0.0 F1400
G1 X60.0 Y40.0
G0 Z10.0

(--- Outer Rounded Rectangle Contour ---)
G0 X0.0 Y10.0
G0 Z2.0
G1 Z-4.0 F400
G1 X0.0 Y70.0 F1500
G2 X10.0 Y80.0 R10.0
G1 X110.0 Y80.0
G2 X120.0 Y70.0 R10.0
G1 X120.0 Y10.0
G2 X110.0 Y0.0 R10.0
G1 X10.0 Y0.0
G2 X0.0 Y10.0 R10.0
G0 Z15.0
M05

(--- OPERATION 2: Tool Change to T2 4mm Drill + G83 Peck Drilling ---)
T2 M06 (T2 D=4.0mm HSS Twist Drill)
G43 H2 Z15.0
S8000 M03
G0 X15.0 Y15.0
G98 G83 X15.0 Y15.0 Z-10.0 R2.0 Q3.0 F350
X105.0 Y15.0
X105.0 Y65.0
X15.0 Y65.0
G80
G0 Z20.0
M05
G28 G91 Z0.
G90
G0 X0.0 Y0.0
M30
)";
}

} // namespace rnr::gcode
