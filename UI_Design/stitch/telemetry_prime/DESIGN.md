# Design System Strategy: Tactical Precision

## 1. Overview & Creative North Star
**Creative North Star: "The Sentinel Interface"**

This design system is a departure from consumer-grade "soft" interfaces. It is an unapologetic embrace of high-fidelity, mission-critical aesthetics. We are moving away from the "friendly" web and toward the **Tactical High-Contrast HMI (Human-Machine Interface)**. 

To achieve a premium, custom feel, we break the "template" look through **Extreme Functionalism**. We reject the roundness of modern mobile apps in favor of a brutalist, orthogonal structure. Asymmetry is driven by data priority: the most critical telemetry occupies the largest visual real estate, while secondary labels are minimized to the edge of legibility. This is a system built for "dark mode" by necessity, where light is a resource used only to signal status, action, or failure.

---

## 2. Colors & Tonal Depth

### The "No-Line" Rule
Traditional borders create visual noise in data-dense environments. This design system prohibits the use of 1px solid borders for sectioning. Structural boundaries are defined exclusively through **Surface Tiering**. If two areas must be separated, one must sit on `surface-container-low` while the other occupies `surface`. 

### Surface Hierarchy & Nesting
We treat the screen as a high-precision instrument panel. Depth is not achieved through shadows, but through "carved" or "raised" tonal shifts:
*   **Root Level:** `surface` (#10141a) – The vast, empty vacuum of the interface.
*   **Component Base:** `surface-container` (#1c2026) – The primary housing for data clusters.
*   **Active Focus:** `surface-container-highest` (#31353c) – Used for active terminal inputs or focused telemetry.

### Tactical Accents
*   **Primary (Phosphor Green):** `primary-container` (#00ff88). Use this only for active states, successful pings, and "Go" conditions.
*   **Secondary (Radar Amber):** `secondary-container` (#fdaf00). For warnings, non-critical alerts, and standby modes.
*   **Tertiary (Crimson):** `on-tertiary-container` (#c50032). Reserved strictly for system errors, hard failures, and critical breach alerts.

*Director’s Note: While the prompt mentions "no gradients," we will utilize "Functional Glows." A 1px glow (box-shadow) of `primary` may be used on active indicators to simulate the physical bleed of a CRT phosphor screen.*

---

## 3. Typography: The Dual-Tone Hierarchy

The system employs a strict bifurcated typography scale to separate **Metadata** from **Live Data**.

*   **The Technical Core (JetBrains Mono):** All dynamic values, coordinates, timestamps, and terminal outputs must use JetBrains Mono. This provides the "Data-Dense" feel and ensures character alignment in streaming data.
*   **The Navigational Layer (Inter):** All UI labels, headers, and instructional text use Inter. This provides a clean, neutral balance to the technicality of the mono font.

### Key Scales:
*   **Display/Headline:** `spaceGrotesk`. Used for major sector callouts (e.g., "ORBITAL VELOCITY"). It adds a subtle aerospace-engineering "blueprint" flair.
*   **Label-SM:** `inter` (0.6875rem). Use uppercase with 0.1em tracking for category headers to maximize space.
*   **Body-MD:** `jetbrainsMono` (0.875rem). The workhorse for all telemetry.

---

## 4. Elevation & Depth: Tonal Layering

### The Layering Principle
We do not "float" elements in this system; we "mount" them. 
*   **Level 0:** `surface-container-lowest` (#0a0e14) for background "voids."
*   **Level 1:** `surface` (#10141a) for the main dashboard plane.
*   **Level 2:** `surface-container-low` (#181c22) for nested data modules.

### The "Ghost Border" Fallback
In rare instances where data density is so high that tonal shifts fail, use a **Ghost Border**.
*   **Token:** `outline-variant` (#3b4b3d) at **15% opacity**.
*   **Rule:** It must feel like a hairline etched into glass, not a painted line.

### Zero Roundness
The roundedness scale is locked at **0px**. Any radius greater than 0px compromises the "military-grade" integrity of the system. Sharp corners denote precision and hardware-level stability.

---

## 5. Components

### Buttons (Action Modules)
*   **Primary:** Background `primary-container` (#00ff88), Text `on-primary` (#003919). 0px radius.
*   **Tertiary (Ghost):** No background. `outline` border at 20% opacity. Text is `on-surface`.
*   **State:** On hover, the background should shift to `primary-fixed-dim` to simulate a lamp warming up.

### Data Chips
*   Used for status tags (e.g., [LOCKED], [SEARCHING]). 
*   No background fill. Use a 1px `outline-variant` and mono-spaced text.
*   Color reflects status: Green for `primary`, Amber for `secondary`.

### Inputs & Terminals
*   Background: `surface-container-highest`.
*   Active Cursor: A solid block of `primary-container` (#00ff88) with a 1Hz blink rate.
*   Labels: Always `label-sm` (Inter, Uppercase) positioned exactly 4px above the input field.

### Cards & Telemetry Blocks
*   **Forbid dividers.** Use 24px vertical spacing between modules. 
*   Header areas should have a slightly darker background (`surface-container-low`) than the content area to anchor the data.

---

## 6. Do’s and Don’ts

### Do:
*   **Embrace the Grid:** Align every element to an 8px baseline. Precision is the primary aesthetic.
*   **Use Monospacing for Numbers:** Ensure all changing values (clocks, coordinates) use JetBrains Mono to prevent "jittering" layouts.
*   **Color as Meaning:** Only use Phosphor Green (#00ff88) if an action is required or a state is active. Do not use it for decoration.

### Don't:
*   **No Softness:** Never use border-radius. If it looks "friendly," it is wrong.
*   **No Shadows:** Do not use drop shadows to create depth. Use the Surface Tiering tokens (`lowest` to `highest`).
*   **No Centering:** Avoid center-aligned text. In a control station, information is scanned left-to-right, top-to-bottom. Stick to rigid left-alignment.
*   **No Icons without Labels:** In high-stakes environments, ambiguity is a failure. Every icon must be accompanied by a `label-sm` text string.