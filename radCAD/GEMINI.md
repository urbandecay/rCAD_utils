# radCAD - Blender Drawing Addon

## Mandatory Behavioral Rules
- **No Unauthorized Writing:** NEVER modify or write to any files without an explicit, direct instruction from the user to do so.
- **No Unauthorized Pushing:** NEVER push code to a remote repository unless specifically instructed by the user, and only using the exact commit name provided.
- **Inquiry vs. Directive:** Treat all questions strictly as inquiries for analysis or information. NEVER implement a fix or change based on a reported issue unless a clear directive to perform the task is given.
- **Strict Adherence:** NEVER perform any actions, organizational changes, or "cleanups" that were not explicitly requested. Focus strictly on the task at hand.
- **Minimalist & Non-Technical:** All replies must be as minimal as possible and strictly non-technical.

## Project Overview
radCAD is a Blender addon (4.2.0+) that provides a suite of CAD-style drawing tools for Edit Mode. It allows for precise creation of arcs, circles, ellipses, polygons, lines, and curves directly on mesh surfaces with advanced snapping and inference capabilities.

### Key Technologies
- **Blender Python API (bpy):** Core integration with Blender.
- **BMesh:** For direct mesh manipulation and geometry creation.
- **GPU Module:** Used for high-performance 3D viewport overlays and HUD drawing.
- **Mathutils:** Extensively used for geometric calculations, plane projections, and coordinate transformations.

### Architecture
- **Modal Framework:** Uses a centralized `ModalManager` (`modal_core.py`) that delegates input and logic to specialized tool classes.
- **State Management:** A global `state` dictionary (`modal_state.py`) tracks tool stages, snapping settings, and geometric data.
- **Tool Hierarchy:** Tools inherit from `radCAD_BaseTool` or `SurfaceDrawTool` (`operators/base_tool.py`), implementing a stage-based interaction model (e.g., Stage 0: Pivot, Stage 1: Radius, Stage 2: Finalize).
- **Geometry Creation:** Once a tool finishes, it commits points to the active mesh using BMesh and optionally performs vertex welding (`arc_weld_manager.py`).
- **Snapping Engine:** Custom snapping system (`snapping_utils.py`) supporting vertices, edges, face centers, and axis inference.
- **UI/UX:** N-panel integration (`panel.py`) with categories and SVG icons. HUD overlays (`hud_overlay.py`) provide real-time feedback and text input for precision.

## Building and Running
As this is a Blender addon, there is no "build" step in the traditional sense.

1.  **Installation:**
    - Zip the `radCAD` directory.
    - In Blender, go to `Edit > Preferences > Add-ons > Install...` and select the zip.
    - Enable "radCAD".
2.  **Usage:**
    - Open the 3D Viewport.
    - Press `N` to open the sidebar and locate the `radCAD` tab.
    - Select a tool (e.g., Arc 1 Point) and click in the viewport to start drawing.
3.  **Development Workflow:**
    - Edit the `.py` files in this directory.
    - Use the "Reload Scripts" operator in Blender (`F3 > Reload Scripts`) or restart Blender to see changes.
    - Use the "Clear Stuck Overlays" button in the radCAD panel if drawing handlers persist after an error.

## Development Conventions
- **Modal Lifecycle:** Always ensure `DrawManager` handles are cleared on exit/cancel to prevent "zombie" overlays.
- **Circular Dependencies:** Be cautious with imports. Use lazy imports within functions (as seen in `modal_core.py`) to avoid circular dependency issues between tools and managers.
- **State Persistence:** Tools should reset their local state in `modal_state.py` upon activation but may respect global toggles (like `snap_verts`).
- **Coordinate Systems:** Most calculations are performed in a local "drawing plane" basis (`Xp`, `Yp`, `Zp`) and سپس projected back to world/object space.
- **Snapping:** New tools should utilize `get_snap_data` from the `ModalManager` to maintain consistency with the global snapping engine.

## Directory Structure Highlights
- `operators/`: Contains implementation of all interactive tools.
- `geometry_utils.py` / `tangent_math.py`: Core mathematical engines for shape calculation.
- `hud_overlay.py` / `tool_previews.py`: Graphics code for viewport feedback.
- `snapping_utils.py`: Logic for 3D snapping and axis inference.


I will say md at the end my request to constantly remind you to follow these directions.  
