import { Immutable, PanelExtensionContext, RenderState, SettingsTreeAction, SettingsTreeNodes } from "@foxglove/extension";
import L from "leaflet";

// Inline Leaflet CSS to avoid external file loading issues in extensions
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
      // Simplified viridis approximation
      const r = Math.round(68 + (253 - 68) * t * t);
      const g = Math.round(1 + (231 - 1) * t);
      const b = Math.round(84 + (37 - 84) * t + (140 - 84) * Math.sin(Math.PI * t));
      return `rgb(${r},${g},${b})`;
    }
  }
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
  gpsTopic: "/nav_sat_fix",
  gridTopic: "/field_heatmap",
  detectionTopic: "/firefly_left/detections",
  mapStyle: "satellite",
  gridOpacity: 0.6,
  colorScale: "red-yellow-green",
  autoCenter: true,
  showTrail: true,
  gaussianRadiusMeters: 3,
  maxIntensity: 50,
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

// ---------- panel init ----------

export function initMapPanel(context: PanelExtensionContext): () => void {
  const panelEl = context.panelElement;
  panelEl.style.width = "100%";
  panelEl.style.height = "100%";
  panelEl.style.position = "relative";
  panelEl.style.overflow = "hidden";

  // Inject Leaflet CSS
  const style = document.createElement("style");
  style.textContent = LEAFLET_CSS;
  panelEl.appendChild(style);

  // Map container
  const mapDiv = document.createElement("div");
  mapDiv.style.width = "100%";
  mapDiv.style.height = "100%";
  panelEl.appendChild(mapDiv);

  // Color scale legend
  const legend = document.createElement("div");
  legend.style.cssText = `
    position:absolute; right:12px; top:12px; z-index:1000;
    background:rgba(0,0,0,0.75); border-radius:6px; padding:8px 10px;
    color:#fff; font-size:11px; font-family:sans-serif; pointer-events:none;
  `;
  panelEl.appendChild(legend);

  // Clear heatmap button
  const clearBtn = document.createElement("button");
  clearBtn.textContent = "Clear Heatmap";
  clearBtn.style.cssText = `
    position:absolute; left:12px; bottom:12px; z-index:1000;
    background:rgba(0,0,0,0.75); border:1px solid rgba(255,255,255,0.3);
    border-radius:4px; padding:6px 12px; color:#fff; font-size:12px;
    font-family:sans-serif; cursor:pointer;
  `;
  clearBtn.addEventListener("click", () => {
    heatGrid.clear();
    if (heatmapLayer) {
      heatmapLayer.clearLayers();
    }
  });
  panelEl.appendChild(clearBtn);

  // State
  let settings: PanelSettings = { ...DEFAULT_SETTINGS };
  let map: L.Map | undefined;
  let tileLayer: L.TileLayer | undefined;
  let robotMarker: L.CircleMarker | undefined;
  let trailLine: L.Polyline | undefined;
  const trailCoords: L.LatLng[] = [];
  let gridLayer: L.LayerGroup | undefined;
  let hasCentered = false;
  let lastGpsTopic = settings.gpsTopic;
  let lastGridTopic = settings.gridTopic;

  // Heatmap accumulation grid: key = "lat,lon" quantized to ~1m cells
  const heatGrid = new Map<string, { lat: number; lon: number; count: number }>();
  let heatmapLayer: L.LayerGroup | undefined;
  let currentGpsLat = 0;
  let currentGpsLon = 0;

  // Initialize map
  function initMap() {
    if (map) {
      map.remove();
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
    tileLayer = L.tileLayer(layer.url, { attribution: layer.attribution, maxZoom: 22 }).addTo(map);

    gridLayer = L.layerGroup().addTo(map);
    heatmapLayer = L.layerGroup().addTo(map);

    robotMarker = L.circleMarker([0, 0], {
      radius: 8,
      color: "#fff",
      weight: 2,
      fillColor: "#2196F3",
      fillOpacity: 1,
    });

    trailLine = L.polyline([], { color: "#2196F3", weight: 2, opacity: 0.7 });
    if (settings.showTrail) {
      trailLine.addTo(map);
    }

    updateLegend();
  }

  function updateLegend() {
    const steps = 6;
    // Build legend using DOM methods
    legend.textContent = "";
    const title = document.createElement("div");
    title.style.cssText = "margin-bottom:4px;font-weight:bold;text-align:center;";
    title.textContent = "Detections";
    legend.appendChild(title);

    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      const detCount = Math.round(t * settings.maxIntensity);
      const row = document.createElement("div");
      row.style.cssText = "display:flex;align-items:center;gap:6px;";
      const swatch = document.createElement("div");
      swatch.style.cssText = `width:16px;height:12px;border-radius:2px;background:${valueToColor(t, settings.colorScale)};`;
      const label = document.createElement("span");
      label.textContent = String(detCount);
      row.appendChild(swatch);
      row.appendChild(label);
      legend.appendChild(row);
    }
  }

  function switchTileLayer() {
    if (!map || !tileLayer) return;
    map.removeLayer(tileLayer);
    const layer = TILE_LAYERS[settings.mapStyle] ?? TILE_LAYERS.satellite!;
    tileLayer = L.tileLayer(layer.url, { attribution: layer.attribution, maxZoom: 22 }).addTo(map);
  }

  // Settings
  function handleSettingsAction(action: SettingsTreeAction) {
    if (action.action !== "update" || !action.payload) return;
    const { path, value } = action.payload;
    const field = path[path.length - 1] as keyof PanelSettings | undefined;
    if (!field || !(field in settings)) return;

    (settings as unknown as Record<string, unknown>)[field] = value;

    if (field === "mapStyle") {
      switchTileLayer();
    }
    if (field === "colorScale") {
      updateLegend();
    }
    if (field === "showTrail" && map && trailLine) {
      if (settings.showTrail) {
        trailLine.addTo(map);
      } else {
        trailLine.remove();
      }
    }

    // Re-subscribe if topics changed
    if (field === "gpsTopic" || field === "gridTopic" || field === "detectionTopic") {
      resubscribe();
    }

    context.updatePanelSettingsEditor({
      actionHandler: handleSettingsAction,
      nodes: buildSettingsTree(settings),
    });
    context.saveState(settings);
  }

  // Load saved state
  const savedState = context.initialState as Partial<PanelSettings> | undefined;
  if (savedState) {
    settings = { ...DEFAULT_SETTINGS, ...savedState };
  }

  context.updatePanelSettingsEditor({
    actionHandler: handleSettingsAction,
    nodes: buildSettingsTree(settings),
  });

  initMap();

  // Subscribe to topics
  function resubscribe() {
    const topics: Record<string, { preload: boolean }> = {};
    if (settings.gpsTopic) {
      topics[settings.gpsTopic] = { preload: false };
    }
    if (settings.gridTopic) {
      topics[settings.gridTopic] = { preload: false };
    }
    if (settings.detectionTopic) {
      topics[settings.detectionTopic] = { preload: false };
    }
    context.subscribe(Object.keys(topics));
    lastGpsTopic = settings.gpsTopic;
    lastGridTopic = settings.gridTopic;
  }

  resubscribe();

  // Handle messages
  context.onRender = (renderState: Immutable<RenderState>, done: () => void) => {
    if (!map) {
      done();
      return;
    }

    // Process incoming messages
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

    // Invalidate map size on render (panel may have resized)
    map.invalidateSize();

    done();
  };

  context.watch("currentFrame");
  context.watch("topics");

  function handleGpsMessage(msg: Record<string, unknown>) {
    if (!map || !robotMarker || !trailLine) return;

    // Support both sensor_msgs/NavSatFix and generic {latitude, longitude}
    const lat = (msg.latitude as number) ?? 0;
    const lon = (msg.longitude as number) ?? 0;
    currentGpsLat = lat;
    currentGpsLon = lon;

    if (lat === 0 && lon === 0) return;

    const pos = L.latLng(lat, lon);

    robotMarker.setLatLng(pos);
    if (!map.hasLayer(robotMarker)) {
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

    // Expected format: { cells: [{lat, lon, value, size?},...] }
    // or { data: [{lat, lon, value, size?},...] }
    const cells = (msg.cells ?? msg.data) as
      | Array<{ lat: number; lon: number; value: number; size?: number }>
      | undefined;

    if (!Array.isArray(cells)) return;

    for (const cell of cells) {
      const halfSize = (cell.size ?? 0.00005) / 2; // ~5m default
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
    if (currentGpsLat === 0 && currentGpsLon === 0) return; // No GPS fix yet

    // vision_msgs/Detection2DArray has a `detections` array
    const detections = msg.detections as Array<Record<string, unknown>> | undefined;
    if (!Array.isArray(detections) || detections.length === 0) return;

    // Quantize GPS to ~1m grid cells (~0.00001 deg = 1.1m)
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

    renderHeatmap();
  }

  function renderHeatmap() {
    if (!map || !heatmapLayer) return;
    heatmapLayer.clearLayers();

    for (const cell of heatGrid.values()) {
      const intensity = Math.min(cell.count / settings.maxIntensity, 1);
      if (intensity <= 0) continue;

      // Outer ring (low opacity, wide)
      L.circle([cell.lat, cell.lon], {
        radius: settings.gaussianRadiusMeters * 1.5,
        color: "transparent",
        fillColor: valueToColor(intensity, settings.colorScale),
        fillOpacity: intensity * settings.gridOpacity * 0.3,
        weight: 0,
      }).addTo(heatmapLayer);

      // Middle ring
      L.circle([cell.lat, cell.lon], {
        radius: settings.gaussianRadiusMeters,
        color: "transparent",
        fillColor: valueToColor(intensity, settings.colorScale),
        fillOpacity: intensity * settings.gridOpacity * 0.6,
        weight: 0,
      }).addTo(heatmapLayer);

      // Inner core (high opacity, tight)
      L.circle([cell.lat, cell.lon], {
        radius: settings.gaussianRadiusMeters * 0.5,
        color: "transparent",
        fillColor: valueToColor(intensity, settings.colorScale),
        fillOpacity: intensity * settings.gridOpacity,
        weight: 0,
      }).addTo(heatmapLayer);
    }
  }

  // Cleanup
  return () => {
    if (map) {
      map.remove();
      map = undefined;
    }
  };
}
