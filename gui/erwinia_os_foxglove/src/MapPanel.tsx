import { Immutable, PanelExtensionContext, RenderState, SettingsTreeAction, SettingsTreeNodes } from "@foxglove/extension";
import L from "leaflet";
import { CachedTileLayer, downloadTilesForArea } from "./CachedTileLayer";

// ---------- Inline Leaflet CSS + custom HUD styles ----------

const LEAFLET_CSS = `
.leaflet-pane,.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow,.leaflet-tile-container,.leaflet-pane>svg,.leaflet-pane>canvas,.leaflet-zoom-box,.leaflet-image-layer,.leaflet-layer{position:absolute;left:0;top:0}
.leaflet-container{overflow:hidden;font:12px/1.5 "Helvetica Neue",Arial,Helvetica,sans-serif}
.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow{-webkit-user-select:none;-moz-user-select:none;user-select:none;-webkit-user-drag:none}
.leaflet-tile{filter:inherit;visibility:hidden}
.leaflet-tile-loaded{visibility:inherit}
.leaflet-zoom-anim .leaflet-zoom-animated{-webkit-transition:-webkit-transform .25s cubic-bezier(0,0,.25,1);-moz-transition:-moz-transform .25s cubic-bezier(0,0,.25,1);transition:transform .25s cubic-bezier(0,0,.25,1)}
.leaflet-tile-container{pointer-events:none}
.leaflet-zoom-anim .leaflet-tile,.leaflet-touching .leaflet-tile{-webkit-transition:none;-moz-transition:none;transition:none}
.leaflet-control{position:relative;z-index:800;pointer-events:visiblePainted;pointer-events:auto}
.leaflet-top,.leaflet-bottom{position:absolute;z-index:1000;pointer-events:none}
.leaflet-top{top:0}.leaflet-right{right:0}.leaflet-bottom{bottom:0}.leaflet-left{left:0}
.leaflet-control{float:left;clear:both}.leaflet-right .leaflet-control{float:right}
.leaflet-top .leaflet-control{margin-top:10px}.leaflet-bottom .leaflet-control{margin-bottom:10px}
.leaflet-left .leaflet-control{margin-left:10px}.leaflet-right .leaflet-control{margin-right:10px}
.leaflet-control-zoom a,.leaflet-control-layers-toggle{background-position:50% 50%;background-repeat:no-repeat;display:block}
.leaflet-control-zoom a{width:30px;height:30px;text-align:center;text-decoration:none;color:#000;background-color:#fff;border-bottom:1px solid #ccc;line-height:30px;font:bold 18px 'Lucida Console',Monaco,monospace}
.leaflet-control-zoom a:hover{background-color:#f4f4f4}
.leaflet-control-zoom-in{border-top-left-radius:4px;border-top-right-radius:4px}
.leaflet-control-zoom-out{border-bottom-left-radius:4px;border-bottom-right-radius:4px}
.leaflet-bar{box-shadow:0 1px 5px rgba(0,0,0,.65);border-radius:4px}
.leaflet-bar a{background-color:#fff;border-bottom:1px solid #ccc;width:26px;height:26px;line-height:26px;display:block;text-align:center;text-decoration:none;color:#000}
.leaflet-bar a:hover{background-color:#f4f4f4}
.leaflet-control-attribution{background:#fff;background:rgba(255,255,255,.7);margin:0;padding:0 5px}
.leaflet-control-attribution a{text-decoration:none}
.leaflet-popup-pane{z-index:700}.leaflet-map-pane{z-index:400}
.leaflet-tile-pane{z-index:200}.leaflet-overlay-pane{z-index:400}
.leaflet-shadow-pane{z-index:500}.leaflet-marker-pane{z-index:600}
.leaflet-tooltip-pane{z-index:650}
.leaflet-zoom-box{border:2px dotted #38f;background:rgba(255,255,255,.5)}
`;

const HUD_CSS = `
@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;500;600;700&family=Outfit:wght@300;400;500;600;700&display=swap');

:root {
  --hud-bg: rgba(8, 12, 21, 0.82);
  --hud-bg-solid: rgba(8, 12, 21, 0.95);
  --hud-border: rgba(56, 189, 248, 0.2);
  --hud-border-bright: rgba(56, 189, 248, 0.45);
  --hud-accent: #38bdf8;
  --hud-accent-dim: rgba(56, 189, 248, 0.6);
  --hud-green: #34d399;
  --hud-green-dim: rgba(52, 211, 153, 0.5);
  --hud-amber: #fbbf24;
  --hud-red: #f87171;
  --hud-text: #e2e8f0;
  --hud-text-dim: rgba(148, 163, 184, 0.8);
  --hud-mono: 'JetBrains Mono', 'SF Mono', 'Fira Code', monospace;
  --hud-sans: 'Outfit', 'SF Pro Display', system-ui, sans-serif;
  --hud-glow: 0 0 12px rgba(56, 189, 248, 0.15), 0 0 4px rgba(56, 189, 248, 0.1);
  --hud-glow-green: 0 0 12px rgba(52, 211, 153, 0.2);
}

/* Override Leaflet zoom controls */
.erwinia-map .leaflet-bar {
  background: var(--hud-bg-solid) !important;
  border: 1px solid var(--hud-border) !important;
  border-radius: 6px !important;
  box-shadow: var(--hud-glow) !important;
  overflow: hidden;
}
.erwinia-map .leaflet-bar a {
  background: transparent !important;
  color: var(--hud-accent) !important;
  border-bottom: 1px solid var(--hud-border) !important;
  font-family: var(--hud-mono) !important;
  font-size: 16px !important;
  width: 32px !important;
  height: 32px !important;
  line-height: 32px !important;
  transition: background 0.15s ease, color 0.15s ease;
}
.erwinia-map .leaflet-bar a:hover {
  background: rgba(56, 189, 248, 0.1) !important;
  color: #fff !important;
}
.erwinia-map .leaflet-bar a:last-child {
  border-bottom: none !important;
}
.erwinia-map .leaflet-control-attribution {
  background: var(--hud-bg) !important;
  color: var(--hud-text-dim) !important;
  font-family: var(--hud-mono) !important;
  font-size: 9px !important;
  padding: 2px 6px !important;
  border-radius: 3px 0 0 0 !important;
}
.erwinia-map .leaflet-control-attribution a {
  color: var(--hud-accent-dim) !important;
}

@keyframes erwinia-fadein {
  from { opacity: 0; transform: translateY(6px); }
  to { opacity: 1; transform: translateY(0); }
}
@keyframes erwinia-blink {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.3; }
}

/* Status bar */
.erwinia-status-bar {
  position: absolute;
  top: 0; left: 0; right: 0;
  z-index: 1001;
  height: 36px;
  background: linear-gradient(180deg, var(--hud-bg-solid) 0%, rgba(8, 12, 21, 0.6) 100%);
  border-bottom: 1px solid var(--hud-border);
  display: flex;
  align-items: center;
  padding: 0 14px;
  gap: 16px;
  font-family: var(--hud-mono);
  font-size: 11px;
  color: var(--hud-text-dim);
  pointer-events: none;
  animation: erwinia-fadein 0.3s ease-out both;
}
.erwinia-status-bar::after {
  content: '';
  position: absolute;
  bottom: -1px;
  left: 0;
  right: 0;
  height: 1px;
  background: linear-gradient(90deg, transparent 0%, var(--hud-accent) 20%, var(--hud-accent) 80%, transparent 100%);
  opacity: 0.3;
}

.erwinia-brand {
  font-family: var(--hud-sans);
  font-weight: 700;
  font-size: 13px;
  letter-spacing: 2.5px;
  text-transform: uppercase;
  color: var(--hud-accent);
  display: flex;
  align-items: center;
  gap: 7px;
}
.erwinia-brand-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: var(--hud-green);
  box-shadow: var(--hud-glow-green);
}
.erwinia-brand-dot[data-status="disconnected"] {
  background: var(--hud-red);
  box-shadow: 0 0 12px rgba(248, 113, 113, 0.3);
  animation: erwinia-blink 1.5s ease-in-out infinite;
}

.erwinia-status-sep {
  width: 1px;
  height: 16px;
  background: var(--hud-border);
}

.erwinia-coord {
  font-variant-numeric: tabular-nums;
  color: var(--hud-text);
  letter-spacing: 0.3px;
}
.erwinia-coord-label {
  color: var(--hud-text-dim);
  margin-right: 4px;
  font-size: 9px;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.erwinia-status-right {
  margin-left: auto;
  display: flex;
  align-items: center;
  gap: 12px;
}

/* Legend */
.erwinia-legend {
  position: absolute;
  right: 12px;
  top: 52px;
  z-index: 1000;
  background: var(--hud-bg);
  backdrop-filter: blur(12px);
  -webkit-backdrop-filter: blur(12px);
  border: 1px solid var(--hud-border);
  border-radius: 8px;
  padding: 10px 12px;
  font-family: var(--hud-mono);
  font-size: 10px;
  color: var(--hud-text-dim);
  pointer-events: none;
  box-shadow: var(--hud-glow);
  animation: erwinia-fadein 0.4s ease-out 0.15s both;
  min-width: 90px;
}
.erwinia-legend-title {
  font-family: var(--hud-sans);
  font-weight: 600;
  font-size: 10px;
  text-transform: uppercase;
  letter-spacing: 1.5px;
  color: var(--hud-accent);
  margin-bottom: 8px;
  padding-bottom: 6px;
  border-bottom: 1px solid var(--hud-border);
}
.erwinia-legend-gradient {
  width: 100%;
  height: 10px;
  border-radius: 3px;
  margin: 6px 0 4px;
}
.erwinia-legend-range {
  display: flex;
  justify-content: space-between;
  font-variant-numeric: tabular-nums;
}

/* Buttons */
.erwinia-btn {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  padding: 7px 14px;
  background: var(--hud-bg);
  backdrop-filter: blur(12px);
  -webkit-backdrop-filter: blur(12px);
  border: 1px solid var(--hud-border);
  border-radius: 6px;
  color: var(--hud-text);
  font-family: var(--hud-sans);
  font-size: 11px;
  font-weight: 500;
  letter-spacing: 0.3px;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: var(--hud-glow);
  pointer-events: auto;
}
.erwinia-btn:hover {
  background: rgba(56, 189, 248, 0.08);
  border-color: var(--hud-border-bright);
  color: #fff;
  box-shadow: 0 0 20px rgba(56, 189, 248, 0.2), 0 0 6px rgba(56, 189, 248, 0.15);
}
.erwinia-btn:active {
  transform: scale(0.97);
}
.erwinia-btn-icon {
  font-size: 13px;
  opacity: 0.7;
}
.erwinia-btn-primary {
  background: rgba(56, 189, 248, 0.12);
  border-color: var(--hud-border-bright);
  color: var(--hud-accent);
}
.erwinia-btn-primary:hover {
  background: rgba(56, 189, 248, 0.2);
  color: #fff;
}
.erwinia-btn-danger {
  border-color: rgba(248, 113, 113, 0.25);
}
.erwinia-btn-danger:hover {
  background: rgba(248, 113, 113, 0.1);
  border-color: rgba(248, 113, 113, 0.5);
  color: var(--hud-red);
  box-shadow: 0 0 16px rgba(248, 113, 113, 0.15);
}

.erwinia-btn-group {
  position: absolute;
  left: 12px;
  bottom: 12px;
  z-index: 1000;
  display: flex;
  gap: 6px;
  animation: erwinia-fadein 0.4s ease-out 0.2s both;
}

/* Download overlay */
.erwinia-dl-overlay {
  position: absolute;
  left: 50%; top: 50%;
  transform: translate(-50%, -50%);
  z-index: 1001;
  background: var(--hud-bg-solid);
  backdrop-filter: blur(20px);
  -webkit-backdrop-filter: blur(20px);
  border: 1px solid var(--hud-border);
  border-radius: 12px;
  padding: 20px 24px;
  color: var(--hud-text);
  font-family: var(--hud-sans);
  font-size: 13px;
  display: none;
  min-width: 300px;
  box-shadow: 0 16px 48px rgba(0,0,0,0.5), var(--hud-glow);
}
.erwinia-dl-title {
  font-weight: 700;
  font-size: 15px;
  letter-spacing: 1px;
  text-transform: uppercase;
  color: var(--hud-accent);
  margin-bottom: 16px;
  padding-bottom: 10px;
  border-bottom: 1px solid var(--hud-border);
}
.erwinia-dl-label {
  display: block;
  margin-bottom: 10px;
  font-size: 11px;
  text-transform: uppercase;
  letter-spacing: 0.8px;
  color: var(--hud-text-dim);
}
.erwinia-dl-input {
  width: 140px;
  padding: 6px 10px;
  border-radius: 5px;
  border: 1px solid var(--hud-border);
  background: rgba(15, 23, 42, 0.8);
  color: var(--hud-text);
  font-family: var(--hud-mono);
  font-size: 12px;
  margin-left: 8px;
  transition: border-color 0.2s;
  outline: none;
}
.erwinia-dl-input:focus {
  border-color: var(--hud-accent);
  box-shadow: 0 0 0 2px rgba(56, 189, 248, 0.1);
}
.erwinia-dl-slider {
  width: 100%;
  margin-top: 6px;
  accent-color: var(--hud-accent);
}
.erwinia-dl-progress {
  width: 100%;
  height: 4px;
  background: rgba(30, 41, 59, 0.8);
  border-radius: 2px;
  overflow: hidden;
  margin-top: 4px;
}
.erwinia-dl-progress-fill {
  width: 0%;
  height: 100%;
  background: linear-gradient(90deg, var(--hud-accent), var(--hud-green));
  border-radius: 2px;
  transition: width 0.3s ease;
}
.erwinia-dl-progress-text {
  font-family: var(--hud-mono);
  font-size: 10px;
  color: var(--hud-text-dim);
  margin-top: 4px;
}
.erwinia-dl-btnrow {
  display: flex;
  gap: 8px;
  margin-top: 16px;
}
`;

// ---------- color scale utilities ----------

type ColorScale = "viridis" | "red-yellow-green" | "thermal";

function valueToColor(value: number, scale: ColorScale): string {
  const t = Math.max(0, Math.min(1, value));
  switch (scale) {
    case "red-yellow-green": {
      const r = t < 0.5 ? 255 : Math.round(255 * (1 - t) * 2);
      const g = t > 0.5 ? 255 : Math.round(255 * t * 2);
      return `rgb(${r},${g},0)`;
    }
    case "thermal": {
      const r = Math.round(255 * Math.min(1, t * 2));
      const g = Math.round(255 * Math.max(0, Math.min(1, (t - 0.25) * 2)));
      const b = Math.round(255 * Math.max(0, (t - 0.5) * 2));
      return `rgb(${r},${g},${b})`;
    }
    case "viridis":
    default: {
      const r = Math.round(68 + (253 - 68) * t * t);
      const g = Math.round(1 + (231 - 1) * t);
      const b = Math.round(84 + (37 - 84) * t + (140 - 84) * Math.sin(Math.PI * t));
      return `rgb(${r},${g},${b})`;
    }
  }
}

function buildGradientCSS(scale: ColorScale, steps = 8): string {
  const colors: string[] = [];
  for (let i = 0; i <= steps; i++) {
    colors.push(valueToColor(i / steps, scale));
  }
  return `linear-gradient(90deg, ${colors.join(", ")})`;
}

// ---------- tile layer URLs ----------

const TILE_LAYERS: Record<string, { url: string; attribution: string }> = {
  street: {
    url: "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a>',
  },
  satellite: {
    url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attribution: "&copy; Esri",
  },
};

// ---------- settings schema ----------

interface PanelSettings {
  gpsTopic: string;
  gridTopic: string;
  detectionTopic: string;
  mapStyle: "street" | "satellite";
  gridOpacity: number;
  colorScale: ColorScale;
  autoCenter: boolean;
  showTrail: boolean;
  gaussianRadiusMeters: number;
  maxIntensity: number;
}

const DEFAULT_SETTINGS: PanelSettings = {
  gpsTopic: "/ns/navsatfix",
  gridTopic: "/field_heatmap",
  detectionTopic: "/firefly_left/detections",
  mapStyle: "satellite",
  gridOpacity: 0.8,
  colorScale: "thermal",
  autoCenter: true,
  showTrail: true,
  gaussianRadiusMeters: 6,
  maxIntensity: 2,
};

function buildSettingsTree(settings: PanelSettings): SettingsTreeNodes {
  return {
    general: {
      label: "General",
      fields: {
        gpsTopic: {
          label: "GPS Topic",
          input: "string",
          value: settings.gpsTopic,
        },
        gridTopic: {
          label: "Grid Topic",
          input: "string",
          value: settings.gridTopic,
        },
        mapStyle: {
          label: "Map Style",
          input: "select",
          value: settings.mapStyle,
          options: [
            { label: "Street", value: "street" },
            { label: "Satellite", value: "satellite" },
          ],
        },
        autoCenter: {
          label: "Auto-Center on Robot",
          input: "boolean",
          value: settings.autoCenter,
        },
        showTrail: {
          label: "Show Robot Trail",
          input: "boolean",
          value: settings.showTrail,
        },
      },
    },
    overlay: {
      label: "Grid Overlay",
      fields: {
        gridOpacity: {
          label: "Grid Opacity",
          input: "number",
          value: settings.gridOpacity,
          min: 0,
          max: 1,
          step: 0.05,
        },
        colorScale: {
          label: "Color Scale",
          input: "select",
          value: settings.colorScale,
          options: [
            { label: "Red-Yellow-Green", value: "red-yellow-green" },
            { label: "Viridis", value: "viridis" },
            { label: "Thermal", value: "thermal" },
          ],
        },
      },
    },
    detections: {
      label: "Detection Heatmap",
      fields: {
        detectionTopic: {
          label: "Detection Topic",
          input: "string",
          value: settings.detectionTopic,
        },
        gaussianRadiusMeters: {
          label: "Gaussian Radius (m)",
          input: "number",
          value: settings.gaussianRadiusMeters,
          min: 1,
          max: 20,
          step: 0.5,
        },
        maxIntensity: {
          label: "Max Intensity (detections)",
          input: "number",
          value: settings.maxIntensity,
          min: 5,
          max: 500,
          step: 5,
        },
      },
    },
  };
}

// ---------- coordinate formatting ----------

function formatCoord(val: number, pos: string, neg: string): string {
  const dir = val >= 0 ? pos : neg;
  const abs = Math.abs(val);
  const deg = Math.floor(abs);
  const min = Math.floor((abs - deg) * 60);
  const sec = ((abs - deg - min / 60) * 3600).toFixed(2);
  return `${deg}\u00B0${String(min).padStart(2, "0")}'${String(sec).padStart(5, "0")}"${dir}`;
}

// ---------- helper: create element with text ----------

function createIcon(text: string): HTMLSpanElement {
  const span = document.createElement("span");
  span.className = "erwinia-btn-icon";
  span.textContent = text;
  return span;
}

// ---------- panel init ----------

export function initMapPanel(context: PanelExtensionContext): () => void {
  const panelEl = context.panelElement;
  panelEl.style.width = "100%";
  panelEl.style.height = "100%";
  panelEl.style.position = "relative";
  panelEl.style.overflow = "hidden";

  // Inject styles
  const style = document.createElement("style");
  style.textContent = LEAFLET_CSS + HUD_CSS;
  panelEl.appendChild(style);

  // Map container
  const mapDiv = document.createElement("div");
  mapDiv.className = "erwinia-map";
  mapDiv.style.width = "100%";
  mapDiv.style.height = "100%";
  panelEl.appendChild(mapDiv);

  // ========== STATUS BAR ==========
  const statusBar = document.createElement("div");
  statusBar.className = "erwinia-status-bar";

  const brand = document.createElement("div");
  brand.className = "erwinia-brand";
  const brandDot = document.createElement("div");
  brandDot.className = "erwinia-brand-dot";
  brandDot.dataset.status = "disconnected";
  brand.appendChild(brandDot);
  brand.appendChild(document.createTextNode("ERWINIA"));
  statusBar.appendChild(brand);

  const sep1 = document.createElement("div");
  sep1.className = "erwinia-status-sep";
  statusBar.appendChild(sep1);

  // Lat display
  const latDisplay = document.createElement("div");
  latDisplay.className = "erwinia-coord";
  const latLbl = document.createElement("span");
  latLbl.className = "erwinia-coord-label";
  latLbl.textContent = "LAT";
  const latVal = document.createElement("span");
  latVal.textContent = "--\u00B0--'--\"";
  latDisplay.appendChild(latLbl);
  latDisplay.appendChild(latVal);
  statusBar.appendChild(latDisplay);

  // Lon display
  const lonDisplay = document.createElement("div");
  lonDisplay.className = "erwinia-coord";
  const lonLbl = document.createElement("span");
  lonLbl.className = "erwinia-coord-label";
  lonLbl.textContent = "LON";
  const lonVal = document.createElement("span");
  lonVal.textContent = "--\u00B0--'--\"";
  lonDisplay.appendChild(lonLbl);
  lonDisplay.appendChild(lonVal);
  statusBar.appendChild(lonDisplay);

  const sep2 = document.createElement("div");
  sep2.className = "erwinia-status-sep";
  statusBar.appendChild(sep2);

  // Alt display
  const altDisplay = document.createElement("div");
  altDisplay.className = "erwinia-coord";
  const altLbl = document.createElement("span");
  altLbl.className = "erwinia-coord-label";
  altLbl.textContent = "ALT";
  const altVal = document.createElement("span");
  altVal.textContent = "--.-m";
  altDisplay.appendChild(altLbl);
  altDisplay.appendChild(altVal);
  statusBar.appendChild(altDisplay);

  // Right side: trail count, detection count
  const statusRight = document.createElement("div");
  statusRight.className = "erwinia-status-right";
  const trailCountEl = document.createElement("div");
  trailCountEl.style.cssText = "font-variant-numeric:tabular-nums;color:var(--hud-accent-dim)";
  trailCountEl.textContent = "TRAIL 0";
  statusRight.appendChild(trailCountEl);

  const sep3 = document.createElement("div");
  sep3.className = "erwinia-status-sep";
  statusRight.appendChild(sep3);

  const detCountEl = document.createElement("div");
  detCountEl.style.cssText = "font-variant-numeric:tabular-nums;color:var(--hud-green-dim)";
  detCountEl.textContent = "DET 0";
  statusRight.appendChild(detCountEl);
  statusBar.appendChild(statusRight);

  panelEl.appendChild(statusBar);

  // ========== LEGEND ==========
  const legend = document.createElement("div");
  legend.className = "erwinia-legend";
  panelEl.appendChild(legend);

  // ========== BUTTONS ==========
  const btnGroup = document.createElement("div");
  btnGroup.className = "erwinia-btn-group";

  const clearBtn = document.createElement("button");
  clearBtn.className = "erwinia-btn erwinia-btn-danger";
  clearBtn.appendChild(createIcon("\u2715"));
  clearBtn.appendChild(document.createTextNode(" Clear Heatmap"));
  clearBtn.addEventListener("click", () => {
    heatGrid.clear();
    totalDetections = 0;
    detCountEl.textContent = "DET 0";
    if (heatmapLayer) {
      heatmapLayer.clearLayers();
    }
  });
  btnGroup.appendChild(clearBtn);

  const openDlBtn = document.createElement("button");
  openDlBtn.className = "erwinia-btn erwinia-btn-primary";
  openDlBtn.appendChild(createIcon("\u2B73"));
  openDlBtn.appendChild(document.createTextNode(" Cache Tiles"));
  openDlBtn.addEventListener("click", () => {
    if (currentGpsLat !== 0 || currentGpsLon !== 0) {
      latInput.value = currentGpsLat.toFixed(6);
      lonInput.value = currentGpsLon.toFixed(6);
    }
    downloadOverlay.style.display = downloadOverlay.style.display === "none" ? "block" : "none";
  });
  btnGroup.appendChild(openDlBtn);
  panelEl.appendChild(btnGroup);

  // ========== DOWNLOAD OVERLAY ==========
  const downloadOverlay = document.createElement("div");
  downloadOverlay.className = "erwinia-dl-overlay";

  const dlTitle = document.createElement("div");
  dlTitle.className = "erwinia-dl-title";
  dlTitle.textContent = "Cache Field Tiles";
  downloadOverlay.appendChild(dlTitle);

  // Lat input
  const latLabel = document.createElement("label");
  latLabel.className = "erwinia-dl-label";
  latLabel.textContent = "Latitude ";
  const latInput = document.createElement("input");
  latInput.type = "number";
  latInput.step = "0.0001";
  latInput.className = "erwinia-dl-input";
  latInput.value = "40.4400";
  latLabel.appendChild(latInput);
  downloadOverlay.appendChild(latLabel);

  // Lon input
  const lonLabel = document.createElement("label");
  lonLabel.className = "erwinia-dl-label";
  lonLabel.textContent = "Longitude ";
  const lonInput = document.createElement("input");
  lonInput.type = "number";
  lonInput.step = "0.0001";
  lonInput.className = "erwinia-dl-input";
  lonInput.value = "-79.9400";
  lonLabel.appendChild(lonInput);
  downloadOverlay.appendChild(lonLabel);

  // Size slider
  const sizeLabel = document.createElement("label");
  sizeLabel.className = "erwinia-dl-label";
  const sizeText = document.createElement("span");
  sizeText.textContent = "Coverage  1 km \u00D7 1 km";
  sizeLabel.appendChild(sizeText);
  sizeLabel.appendChild(document.createElement("br"));
  const sizeSlider = document.createElement("input");
  sizeSlider.type = "range";
  sizeSlider.min = "1";
  sizeSlider.max = "5";
  sizeSlider.step = "0.5";
  sizeSlider.value = "1";
  sizeSlider.className = "erwinia-dl-slider";
  sizeSlider.addEventListener("input", () => {
    sizeText.textContent = `Coverage  ${sizeSlider.value} km \u00D7 ${sizeSlider.value} km`;
  });
  sizeLabel.appendChild(sizeSlider);
  downloadOverlay.appendChild(sizeLabel);

  // Progress
  const progressContainer = document.createElement("div");
  progressContainer.style.display = "none";
  progressContainer.style.marginTop = "12px";
  const progressBar = document.createElement("div");
  progressBar.className = "erwinia-dl-progress";
  const progressFill = document.createElement("div");
  progressFill.className = "erwinia-dl-progress-fill";
  progressBar.appendChild(progressFill);
  progressContainer.appendChild(progressBar);
  const progressText = document.createElement("div");
  progressText.className = "erwinia-dl-progress-text";
  progressContainer.appendChild(progressText);
  downloadOverlay.appendChild(progressContainer);

  // Buttons
  const btnRow = document.createElement("div");
  btnRow.className = "erwinia-dl-btnrow";

  const dlBtn = document.createElement("button");
  dlBtn.className = "erwinia-btn erwinia-btn-primary";
  dlBtn.style.flex = "1";
  dlBtn.textContent = "Download";
  dlBtn.addEventListener("click", async () => {
    const lat = parseFloat(latInput.value);
    const lon = parseFloat(lonInput.value);
    const sizeKm = parseFloat(sizeSlider.value);
    if (isNaN(lat) || isNaN(lon)) return;

    dlBtn.disabled = true;
    dlBtn.textContent = "Downloading\u2026";
    dlBtn.style.opacity = "0.6";
    progressContainer.style.display = "block";

    await downloadTilesForArea(lat, lon, sizeKm, (done, total) => {
      const pct = Math.round((done / total) * 100);
      progressFill.style.width = `${pct}%`;
      progressText.textContent = `${done} / ${total} tiles`;
    });

    dlBtn.disabled = false;
    dlBtn.textContent = "Download";
    dlBtn.style.opacity = "1";
    progressText.textContent = "Complete \u2713";
  });
  btnRow.appendChild(dlBtn);

  const closeBtn = document.createElement("button");
  closeBtn.className = "erwinia-btn";
  closeBtn.style.flex = "1";
  closeBtn.textContent = "Close";
  closeBtn.addEventListener("click", () => {
    downloadOverlay.style.display = "none";
  });
  btnRow.appendChild(closeBtn);
  downloadOverlay.appendChild(btnRow);
  panelEl.appendChild(downloadOverlay);

  // ========== STATE ==========
  let settings: PanelSettings = { ...DEFAULT_SETTINGS };
  let map: L.Map | undefined;
  let tileLayer: L.TileLayer | undefined;
  let robotMarker: L.CircleMarker | undefined;
  let robotPulse: L.CircleMarker | undefined;
  let trailLine: L.Polyline | undefined;
  const trailCoords: L.LatLng[] = [];
  let gridLayer: L.LayerGroup | undefined;
  let hasCentered = false;
  let lastGpsTopic = settings.gpsTopic;
  let lastGridTopic = settings.gridTopic;

  const heatGrid = new Map<string, { lat: number; lon: number; count: number }>();
  let heatmapLayer: L.LayerGroup | undefined;
  let currentGpsLat = 0;
  let currentGpsLon = 0;
  let currentAlt = 0;
  let totalDetections = 0;

  // Pulse animation via timer
  let pulseTimer: ReturnType<typeof setInterval> | undefined;
  let pulsePhase = 0;

  // ========== MAP INIT ==========
  function initMap() {
    if (map) {
      map.remove();
    }
    if (pulseTimer != null) {
      clearInterval(pulseTimer);
    }
    hasCentered = false;
    trailCoords.length = 0;
    heatGrid.clear();

    map = L.map(mapDiv, {
      center: [0, 0],
      zoom: 18,
      zoomControl: true,
      attributionControl: true,
    });

    const layer = TILE_LAYERS[settings.mapStyle] ?? TILE_LAYERS.satellite!;
    tileLayer = new CachedTileLayer(layer.url, { attribution: layer.attribution, maxZoom: 22 }).addTo(map);

    gridLayer = L.layerGroup().addTo(map);
    heatmapLayer = L.layerGroup().addTo(map);

    // Robot marker: bright core with outer glow ring
    robotMarker = L.circleMarker([0, 0], {
      radius: 7,
      color: "#38bdf8",
      weight: 2.5,
      fillColor: "#0ea5e9",
      fillOpacity: 1,
    });

    // Pulsing outer ring
    robotPulse = L.circleMarker([0, 0], {
      radius: 7,
      color: "#38bdf8",
      weight: 1.5,
      fillColor: "transparent",
      fillOpacity: 0,
      opacity: 0.6,
    });

    // Animate pulse
    pulsePhase = 0;
    pulseTimer = setInterval(() => {
      if (!robotPulse) return;
      pulsePhase = (pulsePhase + 1) % 60;
      const t = pulsePhase / 60;
      const r = 7 + t * 18;
      const o = 0.6 * (1 - t);
      robotPulse.setRadius(r);
      robotPulse.setStyle({ opacity: o });
    }, 33);

    trailLine = L.polyline([], {
      color: "#38bdf8",
      weight: 2,
      opacity: 0.5,
      dashArray: "6, 4",
    });
    if (settings.showTrail) {
      trailLine.addTo(map);
    }

    updateLegend();
  }

  function updateLegend() {
    legend.textContent = "";

    const title = document.createElement("div");
    title.className = "erwinia-legend-title";
    title.textContent = "Detections";
    legend.appendChild(title);

    // Gradient bar
    const gradBar = document.createElement("div");
    gradBar.className = "erwinia-legend-gradient";
    gradBar.style.background = buildGradientCSS(settings.colorScale);
    legend.appendChild(gradBar);

    // Range labels
    const range = document.createElement("div");
    range.className = "erwinia-legend-range";
    const minLabel = document.createElement("span");
    minLabel.textContent = "0";
    const maxLabel = document.createElement("span");
    maxLabel.textContent = String(settings.maxIntensity);
    range.appendChild(minLabel);
    range.appendChild(maxLabel);
    legend.appendChild(range);
  }

  function switchTileLayer() {
    if (!map || !tileLayer) return;
    map.removeLayer(tileLayer);
    const layer = TILE_LAYERS[settings.mapStyle] ?? TILE_LAYERS.satellite!;
    tileLayer = new CachedTileLayer(layer.url, { attribution: layer.attribution, maxZoom: 22 }).addTo(map);
  }

  // ========== SETTINGS ==========
  function handleSettingsAction(action: SettingsTreeAction) {
    if (action.action !== "update" || !action.payload) return;
    const { path, value } = action.payload;
    const field = path[path.length - 1] as keyof PanelSettings | undefined;
    if (!field || !(field in settings)) return;

    (settings as unknown as Record<string, unknown>)[field] = value;

    if (field === "mapStyle") {
      switchTileLayer();
    }
    if (field === "colorScale" || field === "maxIntensity") {
      updateLegend();
    }
    if (field === "showTrail" && map && trailLine) {
      if (settings.showTrail) {
        trailLine.addTo(map);
      } else {
        trailLine.remove();
      }
    }

    if (field === "gpsTopic" || field === "gridTopic" || field === "detectionTopic") {
      resubscribe();
    }

    context.updatePanelSettingsEditor({
      actionHandler: handleSettingsAction,
      nodes: buildSettingsTree(settings),
    });
    context.saveState(settings);
  }

  const savedState = context.initialState as Partial<PanelSettings> | undefined;
  if (savedState) {
    settings = { ...DEFAULT_SETTINGS, ...savedState };
  }

  context.updatePanelSettingsEditor({
    actionHandler: handleSettingsAction,
    nodes: buildSettingsTree(settings),
  });

  initMap();

  // ========== SUBSCRIPTIONS ==========
  function resubscribe() {
    const topics: string[] = [];
    if (settings.gpsTopic) topics.push(settings.gpsTopic);
    if (settings.gridTopic) topics.push(settings.gridTopic);
    if (settings.detectionTopic) topics.push(settings.detectionTopic);
    context.subscribe(topics);
    lastGpsTopic = settings.gpsTopic;
    lastGridTopic = settings.gridTopic;
  }

  resubscribe();

  // ========== MESSAGE HANDLERS ==========
  context.onRender = (renderState: Immutable<RenderState>, done: () => void) => {
    if (!map) {
      done();
      return;
    }

    if (renderState.currentFrame) {
      for (const msg of renderState.currentFrame) {
        const topic = msg.topic;
        if (topic === settings.gpsTopic) {
          handleGpsMessage(msg.message as Record<string, unknown>);
        } else if (topic === settings.gridTopic) {
          handleGridMessage(msg.message as Record<string, unknown>);
        } else if (topic === settings.detectionTopic) {
          handleDetectionMessage(msg.message as Record<string, unknown>);
        }
      }
    }

    map.invalidateSize();
    done();
  };

  context.watch("currentFrame");
  context.watch("topics");

  function handleGpsMessage(msg: Record<string, unknown>) {
    if (!map || !robotMarker || !robotPulse || !trailLine) return;

    const lat = (msg.latitude as number) ?? 0;
    const lon = (msg.longitude as number) ?? 0;
    const alt = (msg.altitude as number) ?? 0;
    currentGpsLat = lat;
    currentGpsLon = lon;
    currentAlt = alt;

    if (lat === 0 && lon === 0) return;

    // Update status bar
    brandDot.dataset.status = "connected";
    latVal.textContent = formatCoord(lat, "N", "S");
    lonVal.textContent = formatCoord(lon, "E", "W");
    altVal.textContent = `${alt.toFixed(1)}m`;
    trailCountEl.textContent = `TRAIL ${trailCoords.length}`;

    const pos = L.latLng(lat, lon);

    robotMarker.setLatLng(pos);
    robotPulse.setLatLng(pos);
    if (!map.hasLayer(robotMarker)) {
      robotPulse.addTo(map);
      robotMarker.addTo(map);
    }

    // Trail
    trailCoords.push(pos);
    if (trailCoords.length > 5000) {
      trailCoords.shift();
    }
    trailLine.setLatLngs(trailCoords);

    // Auto-center
    if (settings.autoCenter && !hasCentered) {
      map.setView(pos, 18);
      hasCentered = true;
    } else if (settings.autoCenter) {
      map.panTo(pos);
    }
  }

  function handleGridMessage(msg: Record<string, unknown>) {
    if (!map || !gridLayer) return;
    gridLayer.clearLayers();

    const cells = (msg.cells ?? msg.data) as
      | Array<{ lat: number; lon: number; value: number; size?: number }>
      | undefined;

    if (!Array.isArray(cells)) return;

    for (const cell of cells) {
      const halfSize = (cell.size ?? 0.00005) / 2;
      const bounds: L.LatLngBoundsExpression = [
        [cell.lat - halfSize, cell.lon - halfSize],
        [cell.lat + halfSize, cell.lon + halfSize],
      ];
      const color = valueToColor(cell.value, settings.colorScale);
      L.rectangle(bounds, {
        color: "transparent",
        fillColor: color,
        fillOpacity: settings.gridOpacity,
        weight: 0,
      }).addTo(gridLayer);
    }
  }

  function handleDetectionMessage(msg: Record<string, unknown>) {
    if (!map || !heatmapLayer) return;
    if (currentGpsLat === 0 && currentGpsLon === 0) return;

    const detections = msg.detections as Array<Record<string, unknown>> | undefined;
    if (!Array.isArray(detections) || detections.length === 0) return;

    const resolution = 0.00001;
    const qLat = Math.round(currentGpsLat / resolution) * resolution;
    const qLon = Math.round(currentGpsLon / resolution) * resolution;
    const key = `${qLat.toFixed(6)},${qLon.toFixed(6)}`;

    const existing = heatGrid.get(key);
    if (existing) {
      existing.count += detections.length;
    } else {
      heatGrid.set(key, { lat: qLat, lon: qLon, count: detections.length });
    }

    totalDetections += detections.length;
    detCountEl.textContent = `DET ${totalDetections}`;

    renderHeatmap();
  }

  function renderHeatmap() {
    if (!map || !heatmapLayer) return;
    heatmapLayer.clearLayers();

    for (const cell of heatGrid.values()) {
      const intensity = Math.min(cell.count / settings.maxIntensity, 1);
      if (intensity <= 0) continue;

      const color = valueToColor(intensity, settings.colorScale);
      const pos: L.LatLngExpression = [cell.lat, cell.lon];

      // Wide outer glow - always clearly visible
      L.circle(pos, {
        radius: settings.gaussianRadiusMeters * 2.5,
        color,
        fillColor: color,
        fillOpacity: Math.max(0.25, intensity * 0.6),
        weight: 1.5,
        opacity: Math.max(0.2, intensity * 0.5),
      }).addTo(heatmapLayer);

      // Bold middle ring
      L.circle(pos, {
        radius: settings.gaussianRadiusMeters * 1.2,
        color,
        fillColor: color,
        fillOpacity: Math.max(0.45, intensity * 0.85),
        weight: 2.5,
        opacity: Math.max(0.5, intensity * 0.9),
      }).addTo(heatmapLayer);

      // Bright solid inner core
      L.circle(pos, {
        radius: settings.gaussianRadiusMeters * 0.5,
        color: "#fff",
        fillColor: color,
        fillOpacity: Math.max(0.7, intensity),
        weight: 2,
        opacity: Math.max(0.5, intensity * 0.8),
      }).addTo(heatmapLayer);

      // Center dot marker - always shown
      L.circleMarker(pos, {
        radius: 5,
        color: "#fff",
        weight: 2.5,
        fillColor: color,
        fillOpacity: 1,
        opacity: 1,
      }).addTo(heatmapLayer);
    }
  }

  // ========== CLEANUP ==========
  return () => {
    if (pulseTimer != null) {
      clearInterval(pulseTimer);
    }
    if (map) {
      map.remove();
      map = undefined;
    }
  };
}
