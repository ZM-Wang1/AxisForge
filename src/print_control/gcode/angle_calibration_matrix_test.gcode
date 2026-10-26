; Nozzle angle calibration test matrix
; Firmware profile: generic; bed: rectangular 220 × 220 mm
; Pattern centered at X117.5 Y117.5 (generator X110 Y110 shifted to the 235 mm bed center); origin at bed center: false
; Bed orientation: upright nozzle; all orientations are equivalent at 0° tilt; matrix remains in its standard direction.
; Matrix: 3 flow rows × 3 layer-height columns; 9 specimens; 3 layers per specimen.
; Print order: complete each layer-height column from minimum to maximum; flow increases within each column.
; Nozzle tilt 0 degrees from vertical; rotary axis A rotates about X.
; Selected rotary axis: A (rotation about X).
; Nozzle orientation: trailing edge (no effect at 0° tilt); matrix heading 0°; specimen tilt command is 0° on the selected rotary axis.
; Flow labels are percentages: 100% to 110% in 5% increments.
; Layer-height labels are millimeters: 0.2 to 0.3 mm in 0.05 mm increments; maximum is 1.5 × nozzle-hole diameter.
; Collision-safe minimum height at this angle: 0 mm; physical nozzle tip diameter 0.8 mm.
; Nozzle outlet diameter 0.4 mm sets test-line extrusion width and Slic3r wall spacing.
; Specimen walls: 1; Slic3r centerline spacing varies by layer height; nominal extrusion width 0.4 mm.
; One solid first-layer base connects the matrix labels and all test lines. No individual skirts are printed.
; Base height 0.24 mm; base line width 0.6 mm; label layer 0.2 mm.
; Seven-segment labels: 4 mm high; minimum inter-character clearance 0.3 mm.
; Bead area: A = (w-h)h + pi(h/2)^2. E uses area * distance * flow percentage / filament area.
; Adjacent center spacing = extrusion width - layer height * (1 - pi/4); walls overlap nominally to avoid gaps.
; Travel feedrate fixed at 6000 mm/min.
; Retract 1.2 mm (AxisForge; generated 1.2 mm) at 35 mm/s; Z-hop 0.1 mm.
; Purge path length 63.1 mm per pass (126.2 mm total), with a 0.8 mm return pass, printed before the base.
; Pressure advance fixed at 0.04.
; Review firmware axis direction, bed limits, start/end macros, and nozzle clearance before printing.
G21 ; units in mm
G90 ; absolute XYZ positioning
M83 ; relative extrusion
G92 E0
G28 ; home axes
M140 S50 ; set bed temperature
M190 S50 ; wait for bed
M104 S220 ; set hotend temperature
M109 S220 ; wait for hotend
; Set the fixed pressure-advance value using the command supported by this firmware.

; DOUBLE-PASS PURGE | 63.1 mm each way at 200% flow, separate from matrix base and samples
G0 X85.85 Y104.05 Z2.39 F6000 ; start above the purge line
G0 X85.85 Y104.05 F6000
G0 Z0.24
G1 X185.95 Y104.05 E4.38835614 F1800
G0 X185.95 Y103.25 F6000 ; reposition for return purge pass without extrusion
G1 X85.85 Y103.25 E4.38835614 F1800

; SOLID MATRIX BASE | one connected first layer, no per-line skirts
; Base infill is one continuous snake: alternate row direction and extrude the edge connectors.
M106 S0 ; fan 0%
G1 E-1.2 F2100 ; retract
G0 Z0.34 F6000 ; lift / Z-hop
G0 X86.15 Y131.65 F6000
G0 Z0.24
G1 E1.2 F1800 ; unretract
G1 X148.65 Y131.65 E3.42056932 F1800
G1 X148.65 Y131.1 E0.03010101 F1800
G1 X86.15 Y131.1 E3.42056932 F1800
G1 X86.15 Y130.55 E0.03010101 F1800
G1 X148.65 Y130.55 E3.42056932 F1800
G1 X148.65 Y130.0 E0.03010101 F1800
G1 X86.15 Y130.0 E3.42056932 F1800
G1 X86.15 Y129.45 E0.03010101 F1800
G1 X148.65 Y129.45 E3.42056932 F1800
G1 X148.65 Y128.9 E0.03010101 F1800
G1 X86.15 Y128.9 E3.42056932 F1800
G1 X86.15 Y128.35 E0.03010101 F1800
G1 X148.65 Y128.35 E3.42056932 F1800
G1 X148.65 Y127.8 E0.03010101 F1800
G1 X86.15 Y127.8 E3.42056932 F1800
G1 X86.15 Y127.25 E0.03010101 F1800
G1 X148.65 Y127.25 E3.42056932 F1800
G1 X148.65 Y126.7 E0.03010101 F1800
G1 X86.15 Y126.7 E3.42056932 F1800
G1 X86.15 Y126.15 E0.03010101 F1800
G1 X148.65 Y126.15 E3.42056932 F1800
G1 X148.65 Y125.6 E0.03010101 F1800
G1 X86.15 Y125.6 E3.42056932 F1800
G1 X86.15 Y125.05 E0.03010101 F1800
G1 X148.65 Y125.05 E3.42056932 F1800
G1 X148.65 Y124.5 E0.03010101 F1800
G1 X86.15 Y124.5 E3.42056932 F1800
G1 X86.15 Y123.95 E0.03010101 F1800
G1 X148.65 Y123.95 E3.42056932 F1800
G1 X148.65 Y123.4 E0.03010101 F1800
G1 X86.15 Y123.4 E3.42056932 F1800
G1 X86.15 Y122.85 E0.03010101 F1800
G1 X148.65 Y122.85 E3.42056932 F1800
G1 X148.65 Y122.3 E0.03010101 F1800
G1 X86.15 Y122.3 E3.42056932 F1800
G1 X86.15 Y121.75 E0.03010101 F1800
G1 X148.65 Y121.75 E3.42056932 F1800
G1 X148.65 Y121.2 E0.03010101 F1800
G1 X86.15 Y121.2 E3.42056932 F1800
G1 X86.15 Y120.65 E0.03010101 F1800
G1 X148.65 Y120.65 E3.42056932 F1800
G1 X148.65 Y120.1 E0.03010101 F1800
G1 X86.15 Y120.1 E3.42056932 F1800
G1 X86.15 Y119.55 E0.03010101 F1800
G1 X148.65 Y119.55 E3.42056932 F1800
G1 X148.65 Y119.0 E0.03010101 F1800
G1 X86.15 Y119.0 E3.42056932 F1800
G1 X86.15 Y118.45 E0.03010101 F1800
G1 X148.65 Y118.45 E3.42056932 F1800
G1 X148.65 Y117.9 E0.03010101 F1800
G1 X86.15 Y117.9 E3.42056932 F1800
G1 X86.15 Y117.35 E0.03010101 F1800
G1 X148.65 Y117.35 E3.42056932 F1800
G1 X148.65 Y116.8 E0.03010101 F1800
G1 X86.15 Y116.8 E3.42056932 F1800
G1 X86.15 Y116.25 E0.03010101 F1800
G1 X148.65 Y116.25 E3.42056932 F1800
G1 X148.65 Y115.7 E0.03010101 F1800
G1 X86.15 Y115.7 E3.42056932 F1800
G1 X86.15 Y115.15 E0.03010101 F1800
G1 X148.65 Y115.15 E3.42056932 F1800
G1 X148.65 Y114.6 E0.03010101 F1800
G1 X86.15 Y114.6 E3.42056932 F1800
G1 X86.15 Y114.05 E0.03010101 F1800
G1 X148.65 Y114.05 E3.42056932 F1800
G1 X148.65 Y113.5 E0.03010101 F1800
G1 X86.15 Y113.5 E3.42056932 F1800
G1 X86.15 Y112.95 E0.03010101 F1800
G1 X148.65 Y112.95 E3.42056932 F1800
G1 X148.65 Y112.4 E0.03010101 F1800
G1 X86.15 Y112.4 E3.42056932 F1800
G1 X86.15 Y111.85 E0.03010101 F1800
G1 X148.65 Y111.85 E3.42056932 F1800
G1 X148.65 Y111.3 E0.03010101 F1800
G1 X86.15 Y111.3 E3.42056932 F1800
G1 X86.15 Y110.75 E0.03010101 F1800
G1 X148.65 Y110.75 E3.42056932 F1800
G1 X148.65 Y110.2 E0.03010101 F1800
G1 X86.15 Y110.2 E3.42056932 F1800
G1 X86.15 Y109.65 E0.03010101 F1800
G1 X148.65 Y109.65 E3.42056932 F1800
G1 X148.65 Y109.1 E0.03010101 F1800
G1 X86.15 Y109.1 E3.42056932 F1800
G1 X86.15 Y108.55 E0.03010101 F1800
G1 X148.65 Y108.55 E3.42056932 F1800

; MATRIX LABELS | angle at upper left, heights across the top, flow percentages down the left
; angle label 0°
; Continuous marker strokes for 0°; travel within this label has no retract or Z-hop.
G1 E-1.2 F2100 ; retract
G0 Z0.54 F6000 ; lift / Z-hop
G0 X87.35 Y130.45 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X89.75 Y130.45 E0.0712591 F1800
G0 X89.75 Y130.45 F6000 ; non-extruding move within this label
G1 X89.75 Y128.45 E0.05938258 F1800
G0 X89.75 Y128.45 F6000 ; non-extruding move within this label
G1 X89.75 Y126.45 E0.05938258 F1800
G0 X87.35 Y126.45 F6000 ; non-extruding move within this label
G1 X89.75 Y126.45 E0.0712591 F1800
G0 X87.35 Y128.45 F6000 ; non-extruding move within this label
G1 X87.35 Y126.45 E0.05938258 F1800
G0 X87.35 Y130.45 F6000 ; non-extruding move within this label
G1 X87.35 Y128.45 E0.05938258 F1800
G0 X90.53 Y130.45 F6000 ; non-extruding move within this label
G1 X91.09 Y130.45 E0.01662712 F1800
G0 X91.09 Y130.45 F6000 ; non-extruding move within this label
G1 X91.09 Y129.89 E0.01662712 F1800
G0 X91.09 Y129.89 F6000 ; non-extruding move within this label
G1 X90.53 Y129.89 E0.01662712 F1800
G0 X90.53 Y129.89 F6000 ; non-extruding move within this label
G1 X90.53 Y130.45 E0.01662712 F1800
; height label 0.2 mm
; Continuous marker strokes for 0.2; travel within this label has no retract or Z-hop.
G1 E-1.2 F2100 ; retract
G0 Z0.54 F6000 ; lift / Z-hop
G0 X103.35 Y130.45 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X105.75 Y130.45 E0.0712591 F1800
G0 X105.75 Y130.45 F6000 ; non-extruding move within this label
G1 X105.75 Y128.45 E0.05938258 F1800
G0 X105.75 Y128.45 F6000 ; non-extruding move within this label
G1 X105.75 Y126.45 E0.05938258 F1800
G0 X103.35 Y126.45 F6000 ; non-extruding move within this label
G1 X105.75 Y126.45 E0.0712591 F1800
G0 X103.35 Y128.45 F6000 ; non-extruding move within this label
G1 X103.35 Y126.45 E0.05938258 F1800
G0 X103.35 Y130.45 F6000 ; non-extruding move within this label
G1 X103.35 Y128.45 E0.05938258 F1800
G0 X106.45 Y126.65 F6000 ; non-extruding move within this label
G1 X106.93 Y126.65 E0.01425182 F1800
G0 X107.15 Y130.45 F6000 ; non-extruding move within this label
G1 X109.55 Y130.45 E0.0712591 F1800
G0 X109.55 Y130.45 F6000 ; non-extruding move within this label
G1 X109.55 Y128.45 E0.05938258 F1800
G0 X107.15 Y128.45 F6000 ; non-extruding move within this label
G1 X109.55 Y128.45 E0.0712591 F1800
G0 X107.15 Y128.45 F6000 ; non-extruding move within this label
G1 X107.15 Y126.45 E0.05938258 F1800
G0 X107.15 Y126.45 F6000 ; non-extruding move within this label
G1 X109.55 Y126.45 E0.0712591 F1800
; height label 0.25 mm
; Continuous marker strokes for 0.25; travel within this label has no retract or Z-hop.
G1 E-1.2 F2100 ; retract
G0 Z0.54 F6000 ; lift / Z-hop
G0 X118.8 Y130.45 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X121.2 Y130.45 E0.0712591 F1800
G0 X121.2 Y130.45 F6000 ; non-extruding move within this label
G1 X121.2 Y128.45 E0.05938258 F1800
G0 X121.2 Y128.45 F6000 ; non-extruding move within this label
G1 X121.2 Y126.45 E0.05938258 F1800
G0 X118.8 Y126.45 F6000 ; non-extruding move within this label
G1 X121.2 Y126.45 E0.0712591 F1800
G0 X118.8 Y128.45 F6000 ; non-extruding move within this label
G1 X118.8 Y126.45 E0.05938258 F1800
G0 X118.8 Y130.45 F6000 ; non-extruding move within this label
G1 X118.8 Y128.45 E0.05938258 F1800
G0 X121.9 Y126.65 F6000 ; non-extruding move within this label
G1 X122.38 Y126.65 E0.01425182 F1800
G0 X122.6 Y130.45 F6000 ; non-extruding move within this label
G1 X125.0 Y130.45 E0.0712591 F1800
G0 X125.0 Y130.45 F6000 ; non-extruding move within this label
G1 X125.0 Y128.45 E0.05938258 F1800
G0 X122.6 Y128.45 F6000 ; non-extruding move within this label
G1 X125.0 Y128.45 E0.0712591 F1800
G0 X122.6 Y128.45 F6000 ; non-extruding move within this label
G1 X122.6 Y126.45 E0.05938258 F1800
G0 X122.6 Y126.45 F6000 ; non-extruding move within this label
G1 X125.0 Y126.45 E0.0712591 F1800
G0 X125.7 Y130.45 F6000 ; non-extruding move within this label
G1 X128.1 Y130.45 E0.0712591 F1800
G0 X125.7 Y130.45 F6000 ; non-extruding move within this label
G1 X125.7 Y128.45 E0.05938258 F1800
G0 X125.7 Y128.45 F6000 ; non-extruding move within this label
G1 X128.1 Y128.45 E0.0712591 F1800
G0 X128.1 Y128.45 F6000 ; non-extruding move within this label
G1 X128.1 Y126.45 E0.05938258 F1800
G0 X125.7 Y126.45 F6000 ; non-extruding move within this label
G1 X128.1 Y126.45 E0.0712591 F1800
; height label 0.3 mm
; Continuous marker strokes for 0.3; travel within this label has no retract or Z-hop.
G1 E-1.2 F2100 ; retract
G0 Z0.54 F6000 ; lift / Z-hop
G0 X137.35 Y130.45 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X139.75 Y130.45 E0.0712591 F1800
G0 X139.75 Y130.45 F6000 ; non-extruding move within this label
G1 X139.75 Y128.45 E0.05938258 F1800
G0 X139.75 Y128.45 F6000 ; non-extruding move within this label
G1 X139.75 Y126.45 E0.05938258 F1800
G0 X137.35 Y126.45 F6000 ; non-extruding move within this label
G1 X139.75 Y126.45 E0.0712591 F1800
G0 X137.35 Y128.45 F6000 ; non-extruding move within this label
G1 X137.35 Y126.45 E0.05938258 F1800
G0 X137.35 Y130.45 F6000 ; non-extruding move within this label
G1 X137.35 Y128.45 E0.05938258 F1800
G0 X140.45 Y126.65 F6000 ; non-extruding move within this label
G1 X140.93 Y126.65 E0.01425182 F1800
G0 X141.15 Y130.45 F6000 ; non-extruding move within this label
G1 X143.55 Y130.45 E0.0712591 F1800
G0 X143.55 Y130.45 F6000 ; non-extruding move within this label
G1 X143.55 Y128.45 E0.05938258 F1800
G0 X141.15 Y128.45 F6000 ; non-extruding move within this label
G1 X143.55 Y128.45 E0.0712591 F1800
G0 X143.55 Y128.45 F6000 ; non-extruding move within this label
G1 X143.55 Y126.45 E0.05938258 F1800
G0 X141.15 Y126.45 F6000 ; non-extruding move within this label
G1 X143.55 Y126.45 E0.0712591 F1800
; flow label 100%
; Continuous marker strokes for 100; travel within this label has no retract or Z-hop.
G1 E-1.2 F2100 ; retract
G0 Z0.54 F6000 ; lift / Z-hop
G0 X89.75 Y125.95 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X89.75 Y123.95 E0.05938258 F1800
G0 X89.75 Y123.95 F6000 ; non-extruding move within this label
G1 X89.75 Y121.95 E0.05938258 F1800
G0 X90.45 Y125.95 F6000 ; non-extruding move within this label
G1 X92.85 Y125.95 E0.0712591 F1800
G0 X92.85 Y125.95 F6000 ; non-extruding move within this label
G1 X92.85 Y123.95 E0.05938258 F1800
G0 X92.85 Y123.95 F6000 ; non-extruding move within this label
G1 X92.85 Y121.95 E0.05938258 F1800
G0 X90.45 Y121.95 F6000 ; non-extruding move within this label
G1 X92.85 Y121.95 E0.0712591 F1800
G0 X90.45 Y123.95 F6000 ; non-extruding move within this label
G1 X90.45 Y121.95 E0.05938258 F1800
G0 X90.45 Y125.95 F6000 ; non-extruding move within this label
G1 X90.45 Y123.95 E0.05938258 F1800
G0 X93.55 Y125.95 F6000 ; non-extruding move within this label
G1 X95.95 Y125.95 E0.0712591 F1800
G0 X95.95 Y125.95 F6000 ; non-extruding move within this label
G1 X95.95 Y123.95 E0.05938258 F1800
G0 X95.95 Y123.95 F6000 ; non-extruding move within this label
G1 X95.95 Y121.95 E0.05938258 F1800
G0 X93.55 Y121.95 F6000 ; non-extruding move within this label
G1 X95.95 Y121.95 E0.0712591 F1800
G0 X93.55 Y123.95 F6000 ; non-extruding move within this label
G1 X93.55 Y121.95 E0.05938258 F1800
G0 X93.55 Y125.95 F6000 ; non-extruding move within this label
G1 X93.55 Y123.95 E0.05938258 F1800
; flow label 105%
; Continuous marker strokes for 105; travel within this label has no retract or Z-hop.
G1 E-1.2 F2100 ; retract
G0 Z0.54 F6000 ; lift / Z-hop
G0 X89.75 Y119.55 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X89.75 Y117.55 E0.05938258 F1800
G0 X89.75 Y117.55 F6000 ; non-extruding move within this label
G1 X89.75 Y115.55 E0.05938258 F1800
G0 X90.45 Y119.55 F6000 ; non-extruding move within this label
G1 X92.85 Y119.55 E0.0712591 F1800
G0 X92.85 Y119.55 F6000 ; non-extruding move within this label
G1 X92.85 Y117.55 E0.05938258 F1800
G0 X92.85 Y117.55 F6000 ; non-extruding move within this label
G1 X92.85 Y115.55 E0.05938258 F1800
G0 X90.45 Y115.55 F6000 ; non-extruding move within this label
G1 X92.85 Y115.55 E0.0712591 F1800
G0 X90.45 Y117.55 F6000 ; non-extruding move within this label
G1 X90.45 Y115.55 E0.05938258 F1800
G0 X90.45 Y119.55 F6000 ; non-extruding move within this label
G1 X90.45 Y117.55 E0.05938258 F1800
G0 X93.55 Y119.55 F6000 ; non-extruding move within this label
G1 X95.95 Y119.55 E0.0712591 F1800
G0 X93.55 Y119.55 F6000 ; non-extruding move within this label
G1 X93.55 Y117.55 E0.05938258 F1800
G0 X93.55 Y117.55 F6000 ; non-extruding move within this label
G1 X95.95 Y117.55 E0.0712591 F1800
G0 X95.95 Y117.55 F6000 ; non-extruding move within this label
G1 X95.95 Y115.55 E0.05938258 F1800
G0 X93.55 Y115.55 F6000 ; non-extruding move within this label
G1 X95.95 Y115.55 E0.0712591 F1800
; flow label 110%
; Continuous marker strokes for 110; travel within this label has no retract or Z-hop.
G1 E-1.2 F2100 ; retract
G0 Z0.54 F6000 ; lift / Z-hop
G0 X89.75 Y113.15 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X89.75 Y111.15 E0.05938258 F1800
G0 X89.75 Y111.15 F6000 ; non-extruding move within this label
G1 X89.75 Y109.15 E0.05938258 F1800
G0 X92.85 Y113.15 F6000 ; non-extruding move within this label
G1 X92.85 Y111.15 E0.05938258 F1800
G0 X92.85 Y111.15 F6000 ; non-extruding move within this label
G1 X92.85 Y109.15 E0.05938258 F1800
G0 X93.55 Y113.15 F6000 ; non-extruding move within this label
G1 X95.95 Y113.15 E0.0712591 F1800
G0 X95.95 Y113.15 F6000 ; non-extruding move within this label
G1 X95.95 Y111.15 E0.05938258 F1800
G0 X95.95 Y111.15 F6000 ; non-extruding move within this label
G1 X95.95 Y109.15 E0.05938258 F1800
G0 X93.55 Y109.15 F6000 ; non-extruding move within this label
G1 X95.95 Y109.15 E0.0712591 F1800
G0 X93.55 Y111.15 F6000 ; non-extruding move within this label
G1 X93.55 Y109.15 E0.05938258 F1800
G0 X93.55 Y113.15 F6000 ; non-extruding move within this label
G1 X93.55 Y111.15 E0.05938258 F1800

; TEST MATRIX | each blue line in the printed reference represents one cell specimen
; CELL | nozzle 0 deg | layer 0.2 mm | flow 100% | 1 wall(s) at 0.35708 mm center spacing
M106 S128 ; fan 50%
G1 E-1.2 F2100 ; retract
G0 Z0.54 F6000 ; lift / Z-hop
G0 X98.95 Y123.95 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X113.95 Y123.95 E0.44536938 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.74 F6000 ; lift / Z-hop
G0 X98.95 Y123.95 F6000
G0 Z0.64
G1 E1.2 F1800 ; unretract
G1 X113.95 Y123.95 E0.44536938 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X98.95 Y123.95 F6000
G0 Z0.84
G1 E1.2 F1800 ; unretract
G1 X113.95 Y123.95 E0.44536938 F3600
; CELL | nozzle 0 deg | layer 0.2 mm | flow 105% | 1 wall(s) at 0.35708 mm center spacing
M106 S128 ; fan 50%
G1 E-1.2 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X98.95 Y117.55 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X113.95 Y117.55 E0.46763785 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.74 F6000 ; lift / Z-hop
G0 X98.95 Y117.55 F6000
G0 Z0.64
G1 E1.2 F1800 ; unretract
G1 X113.95 Y117.55 E0.46763785 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X98.95 Y117.55 F6000
G0 Z0.84
G1 E1.2 F1800 ; unretract
G1 X113.95 Y117.55 E0.46763785 F3600
; CELL | nozzle 0 deg | layer 0.2 mm | flow 110% | 1 wall(s) at 0.35708 mm center spacing
M106 S128 ; fan 50%
G1 E-1.2 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X98.95 Y111.15 F6000
G0 Z0.44
G1 E1.2 F1800 ; unretract
G1 X113.95 Y111.15 E0.48990632 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.74 F6000 ; lift / Z-hop
G0 X98.95 Y111.15 F6000
G0 Z0.64
G1 E1.2 F1800 ; unretract
G1 X113.95 Y111.15 E0.48990632 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X98.95 Y111.15 F6000
G0 Z0.84
G1 E1.2 F1800 ; unretract
G1 X113.95 Y111.15 E0.48990632 F3600
; CELL | nozzle 0 deg | layer 0.25 mm | flow 100% | 1 wall(s) at 0.34635 mm center spacing
M106 S128 ; fan 50%
G1 E-1.2 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X115.95 Y123.95 F6000
G0 Z0.49
G1 E1.2 F1800 ; unretract
G1 X130.95 Y123.95 E0.53998277 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.84 F6000 ; lift / Z-hop
G0 X115.95 Y123.95 F6000
G0 Z0.74
G1 E1.2 F1800 ; unretract
G1 X130.95 Y123.95 E0.53998277 F3600
G1 E-1.2 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X115.95 Y123.95 F6000
G0 Z0.99
G1 E1.2 F1800 ; unretract
G1 X130.95 Y123.95 E0.53998277 F3600
; CELL | nozzle 0 deg | layer 0.25 mm | flow 105% | 1 wall(s) at 0.34635 mm center spacing
M106 S128 ; fan 50%
G1 E-1.2 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X115.95 Y117.55 F6000
G0 Z0.49
G1 E1.2 F1800 ; unretract
G1 X130.95 Y117.55 E0.56698191 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.84 F6000 ; lift / Z-hop
G0 X115.95 Y117.55 F6000
G0 Z0.74
G1 E1.2 F1800 ; unretract
G1 X130.95 Y117.55 E0.56698191 F3600
G1 E-1.2 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X115.95 Y117.55 F6000
G0 Z0.99
G1 E1.2 F1800 ; unretract
G1 X130.95 Y117.55 E0.56698191 F3600
; CELL | nozzle 0 deg | layer 0.25 mm | flow 110% | 1 wall(s) at 0.34635 mm center spacing
M106 S128 ; fan 50%
G1 E-1.2 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X115.95 Y111.15 F6000
G0 Z0.49
G1 E1.2 F1800 ; unretract
G1 X130.95 Y111.15 E0.59398105 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.84 F6000 ; lift / Z-hop
G0 X115.95 Y111.15 F6000
G0 Z0.74
G1 E1.2 F1800 ; unretract
G1 X130.95 Y111.15 E0.59398105 F3600
G1 E-1.2 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X115.95 Y111.15 F6000
G0 Z0.99
G1 E1.2 F1800 ; unretract
G1 X130.95 Y111.15 E0.59398105 F3600
; CELL | nozzle 0 deg | layer 0.3 mm | flow 100% | 1 wall(s) at 0.335619 mm center spacing
M106 S128 ; fan 50%
G1 E-1.2 F2100 ; retract
G0 Z1.09 F6000 ; lift / Z-hop
G0 X132.95 Y123.95 F6000
G0 Z0.54
G1 E1.2 F1800 ; unretract
G1 X147.95 Y123.95 E0.62790459 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X132.95 Y123.95 F6000
G0 Z0.84
G1 E1.2 F1800 ; unretract
G1 X147.95 Y123.95 E0.62790459 F3600
G1 E-1.2 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X132.95 Y123.95 F6000
G0 Z1.14
G1 E1.2 F1800 ; unretract
G1 X147.95 Y123.95 E0.62790459 F3600
; CELL | nozzle 0 deg | layer 0.3 mm | flow 105% | 1 wall(s) at 0.335619 mm center spacing
M106 S128 ; fan 50%
G1 E-1.2 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X132.95 Y117.55 F6000
G0 Z0.54
G1 E1.2 F1800 ; unretract
G1 X147.95 Y117.55 E0.65929982 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X132.95 Y117.55 F6000
G0 Z0.84
G1 E1.2 F1800 ; unretract
G1 X147.95 Y117.55 E0.65929982 F3600
G1 E-1.2 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X132.95 Y117.55 F6000
G0 Z1.14
G1 E1.2 F1800 ; unretract
G1 X147.95 Y117.55 E0.65929982 F3600
; CELL | nozzle 0 deg | layer 0.3 mm | flow 110% | 1 wall(s) at 0.335619 mm center spacing
M106 S128 ; fan 50%
G1 E-1.2 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X132.95 Y111.15 F6000
G0 Z0.54
G1 E1.2 F1800 ; unretract
G1 X147.95 Y111.15 E0.69069504 F3600
G1 E-1.2 F2100 ; retract
G0 Z0.94 F6000 ; lift / Z-hop
G0 X132.95 Y111.15 F6000
G0 Z0.84
G1 E1.2 F1800 ; unretract
G1 X147.95 Y111.15 E0.69069504 F3600
G1 E-1.2 F2100 ; retract
G0 Z1.24 F6000 ; lift / Z-hop
G0 X132.95 Y111.15 F6000
G0 Z1.14
G1 E1.2 F1800 ; unretract
G1 X147.95 Y111.15 E0.69069504 F3600
; Default shutdown: turn off hotend and bed
M104 S0 ; turn off hotend
M140 S0 ; turn off bed

; End of nozzle angle calibration test matrix.
