; Nozzle angle calibration test matrix
; Firmware profile: klipper; bed: rectangular 220 × 220 mm
; Pattern centered at X110 Y110; origin at bed center: false
; Bed orientation: upright nozzle; all orientations are equivalent at 0° tilt; matrix remains in its standard direction.
; Matrix: 3 flow rows × 3 layer-height columns; 9 specimens; 3 layers per specimen.
; Print order: complete each layer-height column from minimum to maximum; flow increases within each column.
; Nozzle tilt 0 degrees from vertical; rotary axis A rotates about X.
; Selected rotary axis: A (rotation about X).
; Nozzle orientation: trailing edge (no effect at 0° tilt); matrix heading 0°; specimen tilt command is 0° on the selected rotary axis.
; Flow labels are percentages: 100% to 110% in 5% increments.
; Layer-height labels are millimeters: 0.25 to 0.35 mm in 0.05 mm increments; maximum is 1.5 × nozzle-hole diameter.
; Collision-safe minimum height at this angle: 0 mm; physical nozzle tip diameter 0.8 mm.
; Nozzle outlet diameter 0.4 mm sets test-line extrusion width and Slic3r wall spacing.
; Specimen walls: 1; Slic3r centerline spacing varies by layer height; nominal extrusion width 0.4 mm.
; One solid first-layer base connects the matrix labels and all test lines. No individual skirts are printed.
; Base height 0.24 mm; base line width 0.6 mm; label layer 0.25 mm.
; Seven-segment labels: 4 mm high; minimum inter-character clearance 0.3 mm.
; Bead area: A = (w-h)h + pi(h/2)^2. E uses area * distance * flow percentage / filament area.
; Adjacent center spacing = extrusion width - layer height * (1 - pi/4); walls overlap nominally to avoid gaps.
; Travel feedrate fixed at 6000 mm/min.
; Retract 0.6 mm at 35 mm/s; Z-hop 0.1 mm.
; Purge path length 63.1 mm per pass (126.2 mm total), with a 0.8 mm return pass, printed before the base.
; Pressure advance fixed at 0.04.
; Review firmware axis direction, bed limits, start/end macros, and nozzle clearance before printing.
G21 ; units in mm
G90 ; absolute XYZ positioning
M83 ; relative extrusion
G92 E0
G28 ; home axes
M140 S60 ; set bed temperature
M190 S60 ; wait for bed
M104 S210 ; set hotend temperature
M109 S210 ; wait for hotend
SET_PRESSURE_ADVANCE ADVANCE=0.04

; DOUBLE-PASS PURGE | 63.1 mm each way at 200% flow, separate from matrix base and samples
G0 Z0.34 F6000 ; lift / Z-hop
G0 X78.35 Y96.55 F6000
G0 Z0.24
G1 A0 F6000
G1 X141.45 Y96.55 E4.38835614 F1800
G0 X141.45 Y95.75 F6000 ; reposition for return purge pass without extrusion
G1 A0 F6000
G1 X78.35 Y95.75 E4.38835614 F1800

; SOLID MATRIX BASE | one connected first layer, no per-line skirts
; Base infill is one continuous snake: alternate row direction and extrude the edge connectors.
M106 S0 ; fan 0%
G1 E-0.6 F2100 ; retract
G0 Z0.34 F6000 ; lift / Z-hop
G0 X78.65 Y124.15 F6000
G0 Z0.24
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X141.15 Y124.15 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y123.6 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y123.6 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y123.05 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y123.05 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y122.5 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y122.5 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y121.95 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y121.95 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y121.4 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y121.4 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y120.85 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y120.85 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y120.3 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y120.3 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y119.75 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y119.75 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y119.2 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y119.2 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y118.65 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y118.65 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y118.1 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y118.1 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y117.55 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y117.55 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y117 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y117 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y116.45 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y116.45 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y115.9 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y115.9 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y115.35 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y115.35 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y114.8 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y114.8 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y114.25 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y114.25 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y113.7 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y113.7 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y113.15 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y113.15 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y112.6 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y112.6 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y112.05 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y112.05 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y111.5 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y111.5 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y110.95 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y110.95 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y110.4 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y110.4 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y109.85 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y109.85 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y109.3 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y109.3 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y108.75 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y108.75 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y108.2 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y108.2 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y107.65 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y107.65 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y107.1 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y107.1 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y106.55 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y106.55 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y106 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y106 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y105.45 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y105.45 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y104.9 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y104.9 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y104.35 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y104.35 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y103.8 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y103.8 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y103.25 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y103.25 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y102.7 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y102.7 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y102.15 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y102.15 E3.42056932 F1800
G1 A0 F6000
G1 X141.15 Y101.6 E0.03010101 F1800
G1 A0 F6000
G1 X78.65 Y101.6 E3.42056932 F1800
G1 A0 F6000
G1 X78.65 Y101.05 E0.03010101 F1800
G1 A0 F6000
G1 X141.15 Y101.05 E3.42056932 F1800

; MATRIX LABELS | angle at upper left, heights across the top, flow percentages down the left
; angle label 0°
; Continuous marker strokes for 0°; travel within this label has no retract or Z-hop.
G1 E-0.6 F2100 ; retract
G0 Z0.59 F6000 ; lift / Z-hop
G0 X79.85 Y122.95 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X82.25 Y122.95 E0.08639724 F1800
G0 X82.25 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.25 Y120.95 E0.0719977 F1800
G0 X82.25 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.25 Y118.95 E0.0719977 F1800
G0 X79.85 Y118.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.25 Y118.95 E0.08639724 F1800
G0 X79.85 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X79.85 Y118.95 E0.0719977 F1800
G0 X79.85 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X79.85 Y120.95 E0.0719977 F1800
G0 X83.03 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X83.59 Y122.95 E0.02015936 F1800
G0 X83.59 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X83.59 Y122.39 E0.02015936 F1800
G0 X83.59 Y122.39 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X83.03 Y122.39 E0.02015936 F1800
G0 X83.03 Y122.39 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X83.03 Y122.95 E0.02015936 F1800
; height label 0.25 mm
; Continuous marker strokes for 0.25; travel within this label has no retract or Z-hop.
G1 E-0.6 F2100 ; retract
G0 Z0.59 F6000 ; lift / Z-hop
G0 X94.3 Y122.95 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X96.7 Y122.95 E0.08639724 F1800
G0 X96.7 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X96.7 Y120.95 E0.0719977 F1800
G0 X96.7 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X96.7 Y118.95 E0.0719977 F1800
G0 X94.3 Y118.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X96.7 Y118.95 E0.08639724 F1800
G0 X94.3 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X94.3 Y118.95 E0.0719977 F1800
G0 X94.3 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X94.3 Y120.95 E0.0719977 F1800
G0 X97.4 Y119.15 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X97.88 Y119.15 E0.01727945 F1800
G0 X98.1 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X100.5 Y122.95 E0.08639724 F1800
G0 X100.5 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X100.5 Y120.95 E0.0719977 F1800
G0 X98.1 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X100.5 Y120.95 E0.08639724 F1800
G0 X98.1 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X98.1 Y118.95 E0.0719977 F1800
G0 X98.1 Y118.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X100.5 Y118.95 E0.08639724 F1800
G0 X101.2 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X103.6 Y122.95 E0.08639724 F1800
G0 X101.2 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X101.2 Y120.95 E0.0719977 F1800
G0 X101.2 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X103.6 Y120.95 E0.08639724 F1800
G0 X103.6 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X103.6 Y118.95 E0.0719977 F1800
G0 X101.2 Y118.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X103.6 Y118.95 E0.08639724 F1800
; height label 0.3 mm
; Continuous marker strokes for 0.3; travel within this label has no retract or Z-hop.
G1 E-0.6 F2100 ; retract
G0 Z0.59 F6000 ; lift / Z-hop
G0 X112.85 Y122.95 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X115.25 Y122.95 E0.08639724 F1800
G0 X115.25 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X115.25 Y120.95 E0.0719977 F1800
G0 X115.25 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X115.25 Y118.95 E0.0719977 F1800
G0 X112.85 Y118.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X115.25 Y118.95 E0.08639724 F1800
G0 X112.85 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X112.85 Y118.95 E0.0719977 F1800
G0 X112.85 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X112.85 Y120.95 E0.0719977 F1800
G0 X115.95 Y119.15 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X116.43 Y119.15 E0.01727945 F1800
G0 X116.65 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X119.05 Y122.95 E0.08639724 F1800
G0 X119.05 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X119.05 Y120.95 E0.0719977 F1800
G0 X116.65 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X119.05 Y120.95 E0.08639724 F1800
G0 X119.05 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X119.05 Y118.95 E0.0719977 F1800
G0 X116.65 Y118.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X119.05 Y118.95 E0.08639724 F1800
; height label 0.35 mm
; Continuous marker strokes for 0.35; travel within this label has no retract or Z-hop.
G1 E-0.6 F2100 ; retract
G0 Z0.59 F6000 ; lift / Z-hop
G0 X128.3 Y122.95 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X130.7 Y122.95 E0.08639724 F1800
G0 X130.7 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X130.7 Y120.95 E0.0719977 F1800
G0 X130.7 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X130.7 Y118.95 E0.0719977 F1800
G0 X128.3 Y118.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X130.7 Y118.95 E0.08639724 F1800
G0 X128.3 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X128.3 Y118.95 E0.0719977 F1800
G0 X128.3 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X128.3 Y120.95 E0.0719977 F1800
G0 X131.4 Y119.15 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X131.88 Y119.15 E0.01727945 F1800
G0 X132.1 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X134.5 Y122.95 E0.08639724 F1800
G0 X134.5 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X134.5 Y120.95 E0.0719977 F1800
G0 X132.1 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X134.5 Y120.95 E0.08639724 F1800
G0 X134.5 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X134.5 Y118.95 E0.0719977 F1800
G0 X132.1 Y118.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X134.5 Y118.95 E0.08639724 F1800
G0 X135.2 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X137.6 Y122.95 E0.08639724 F1800
G0 X135.2 Y122.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X135.2 Y120.95 E0.0719977 F1800
G0 X135.2 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X137.6 Y120.95 E0.08639724 F1800
G0 X137.6 Y120.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X137.6 Y118.95 E0.0719977 F1800
G0 X135.2 Y118.95 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X137.6 Y118.95 E0.08639724 F1800
; flow label 100%
; Continuous marker strokes for 100; travel within this label has no retract or Z-hop.
G1 E-0.6 F2100 ; retract
G0 Z0.59 F6000 ; lift / Z-hop
G0 X82.25 Y118.45 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X82.25 Y116.45 E0.0719977 F1800
G0 X82.25 Y116.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.25 Y114.45 E0.0719977 F1800
G0 X82.95 Y118.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y118.45 E0.08639724 F1800
G0 X85.35 Y118.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y116.45 E0.0719977 F1800
G0 X85.35 Y116.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y114.45 E0.0719977 F1800
G0 X82.95 Y114.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y114.45 E0.08639724 F1800
G0 X82.95 Y116.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.95 Y114.45 E0.0719977 F1800
G0 X82.95 Y118.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.95 Y116.45 E0.0719977 F1800
G0 X86.05 Y118.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y118.45 E0.08639724 F1800
G0 X88.45 Y118.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y116.45 E0.0719977 F1800
G0 X88.45 Y116.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y114.45 E0.0719977 F1800
G0 X86.05 Y114.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y114.45 E0.08639724 F1800
G0 X86.05 Y116.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X86.05 Y114.45 E0.0719977 F1800
G0 X86.05 Y118.45 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X86.05 Y116.45 E0.0719977 F1800
; flow label 105%
; Continuous marker strokes for 105; travel within this label has no retract or Z-hop.
G1 E-0.6 F2100 ; retract
G0 Z0.59 F6000 ; lift / Z-hop
G0 X82.25 Y112.05 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X82.25 Y110.05 E0.0719977 F1800
G0 X82.25 Y110.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.25 Y108.05 E0.0719977 F1800
G0 X82.95 Y112.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y112.05 E0.08639724 F1800
G0 X85.35 Y112.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y110.05 E0.0719977 F1800
G0 X85.35 Y110.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y108.05 E0.0719977 F1800
G0 X82.95 Y108.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y108.05 E0.08639724 F1800
G0 X82.95 Y110.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.95 Y108.05 E0.0719977 F1800
G0 X82.95 Y112.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.95 Y110.05 E0.0719977 F1800
G0 X86.05 Y112.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y112.05 E0.08639724 F1800
G0 X86.05 Y112.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X86.05 Y110.05 E0.0719977 F1800
G0 X86.05 Y110.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y110.05 E0.08639724 F1800
G0 X88.45 Y110.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y108.05 E0.0719977 F1800
G0 X86.05 Y108.05 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y108.05 E0.08639724 F1800
; flow label 110%
; Continuous marker strokes for 110; travel within this label has no retract or Z-hop.
G1 E-0.6 F2100 ; retract
G0 Z0.59 F6000 ; lift / Z-hop
G0 X82.25 Y105.65 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X82.25 Y103.65 E0.0719977 F1800
G0 X82.25 Y103.65 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X82.25 Y101.65 E0.0719977 F1800
G0 X85.35 Y105.65 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y103.65 E0.0719977 F1800
G0 X85.35 Y103.65 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X85.35 Y101.65 E0.0719977 F1800
G0 X86.05 Y105.65 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y105.65 E0.08639724 F1800
G0 X88.45 Y105.65 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y103.65 E0.0719977 F1800
G0 X88.45 Y103.65 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y101.65 E0.0719977 F1800
G0 X86.05 Y101.65 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X88.45 Y101.65 E0.08639724 F1800
G0 X86.05 Y103.65 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X86.05 Y101.65 E0.0719977 F1800
G0 X86.05 Y105.65 F6000 ; non-extruding move within this label
G1 A0 F6000
G1 X86.05 Y103.65 E0.0719977 F1800

; TEST MATRIX | each blue line in the printed reference represents one cell specimen
; CELL | nozzle 0 deg | layer 0.25 mm | flow 100% | 1 wall(s) at 0.34635 mm center spacing
M106 S128 ; fan 50%
G1 E-0.6 F2100 ; retract
G0 Z0.59 F6000 ; lift / Z-hop
G0 X91.45 Y116.45 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X106.45 Y116.45 E0.53998277 F1800
G1 E-0.6 F2100 ; retract
G0 Z0.84 F6000 ; lift / Z-hop
G0 X91.45 Y116.45 F6000
G0 Z0.74
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X106.45 Y116.45 E0.53998277 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X91.45 Y116.45 F6000
G0 Z0.99
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X106.45 Y116.45 E0.53998277 F1800
; CELL | nozzle 0 deg | layer 0.25 mm | flow 105% | 1 wall(s) at 0.34635 mm center spacing
M106 S128 ; fan 50%
G1 E-0.6 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X91.45 Y110.05 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X106.45 Y110.05 E0.56698191 F1800
G1 E-0.6 F2100 ; retract
G0 Z0.84 F6000 ; lift / Z-hop
G0 X91.45 Y110.05 F6000
G0 Z0.74
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X106.45 Y110.05 E0.56698191 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X91.45 Y110.05 F6000
G0 Z0.99
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X106.45 Y110.05 E0.56698191 F1800
; CELL | nozzle 0 deg | layer 0.25 mm | flow 110% | 1 wall(s) at 0.34635 mm center spacing
M106 S128 ; fan 50%
G1 E-0.6 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X91.45 Y103.65 F6000
G0 Z0.49
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X106.45 Y103.65 E0.59398105 F1800
G1 E-0.6 F2100 ; retract
G0 Z0.84 F6000 ; lift / Z-hop
G0 X91.45 Y103.65 F6000
G0 Z0.74
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X106.45 Y103.65 E0.59398105 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X91.45 Y103.65 F6000
G0 Z0.99
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X106.45 Y103.65 E0.59398105 F1800
; CELL | nozzle 0 deg | layer 0.3 mm | flow 100% | 1 wall(s) at 0.335619 mm center spacing
M106 S128 ; fan 50%
G1 E-0.6 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X108.45 Y116.45 F6000
G0 Z0.54
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X123.45 Y116.45 E0.62790459 F1800
G1 E-0.6 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X108.45 Y116.45 F6000
G0 Z0.84
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X123.45 Y116.45 E0.62790459 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X108.45 Y116.45 F6000
G0 Z1.14
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X123.45 Y116.45 E0.62790459 F1800
; CELL | nozzle 0 deg | layer 0.3 mm | flow 105% | 1 wall(s) at 0.335619 mm center spacing
M106 S128 ; fan 50%
G1 E-0.6 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X108.45 Y110.05 F6000
G0 Z0.54
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X123.45 Y110.05 E0.65929982 F1800
G1 E-0.6 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X108.45 Y110.05 F6000
G0 Z0.84
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X123.45 Y110.05 E0.65929982 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X108.45 Y110.05 F6000
G0 Z1.14
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X123.45 Y110.05 E0.65929982 F1800
; CELL | nozzle 0 deg | layer 0.3 mm | flow 110% | 1 wall(s) at 0.335619 mm center spacing
M106 S128 ; fan 50%
G1 E-0.6 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X108.45 Y103.65 F6000
G0 Z0.54
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X123.45 Y103.65 E0.69069504 F1800
G1 E-0.6 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X108.45 Y103.65 F6000
G0 Z0.84
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X123.45 Y103.65 E0.69069504 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X108.45 Y103.65 F6000
G0 Z1.14
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X123.45 Y103.65 E0.69069504 F1800
; CELL | nozzle 0 deg | layer 0.35 mm | flow 100% | 1 wall(s) at 0.324889 mm center spacing
M106 S128 ; fan 50%
G1 E-0.6 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X125.45 Y116.45 F6000
G0 Z0.59
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X140.45 Y116.45 E0.70913482 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.04 F6000 ; lift / Z-hop
G0 X125.45 Y116.45 F6000
G0 Z0.94
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X140.45 Y116.45 E0.70913482 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.39 F6000 ; lift / Z-hop
G0 X125.45 Y116.45 F6000
G0 Z1.29
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X140.45 Y116.45 E0.70913482 F1800
; CELL | nozzle 0 deg | layer 0.35 mm | flow 105% | 1 wall(s) at 0.324889 mm center spacing
M106 S128 ; fan 50%
G1 E-0.6 F2100 ; retract
G0 Z1.39 F6000 ; lift / Z-hop
G0 X125.45 Y110.05 F6000
G0 Z0.59
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X140.45 Y110.05 E0.74459156 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.04 F6000 ; lift / Z-hop
G0 X125.45 Y110.05 F6000
G0 Z0.94
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X140.45 Y110.05 E0.74459156 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.39 F6000 ; lift / Z-hop
G0 X125.45 Y110.05 F6000
G0 Z1.29
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X140.45 Y110.05 E0.74459156 F1800
; CELL | nozzle 0 deg | layer 0.35 mm | flow 110% | 1 wall(s) at 0.324889 mm center spacing
M106 S128 ; fan 50%
G1 E-0.6 F2100 ; retract
G0 Z1.39 F6000 ; lift / Z-hop
G0 X125.45 Y103.65 F6000
G0 Z0.59
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X140.45 Y103.65 E0.7800483 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.04 F6000 ; lift / Z-hop
G0 X125.45 Y103.65 F6000
G0 Z0.94
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X140.45 Y103.65 E0.7800483 F1800
G1 E-0.6 F2100 ; retract
G0 Z1.39 F6000 ; lift / Z-hop
G0 X125.45 Y103.65 F6000
G0 Z1.29
G1 E0.6 F1800 ; unretract
G1 A0 F6000
G1 X140.45 Y103.65 E0.7800483 F1800
; Default shutdown: turn off part cooling fan, hotend, and bed
M107 ; turn off part cooling fan
M104 S0 ; turn off hotend
M140 S0 ; turn off bed

; End of nozzle angle calibration test matrix.
