import {
  Immutable,
  PanelExtensionContext,
  RenderState,
  SettingsTreeAction,
  SettingsTreeNodes,
} from "@foxglove/extension";

const SPEED_PANEL_CSS = `
@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;700&family=Outfit:wght@400;500;600;700&display=swap');

:root {
  --speed-bg: radial-gradient(circle at top left, rgba(34, 211, 238, 0.14), transparent 38%), linear-gradient(180deg, #09101c 0%, #060b13 100%);
  --speed-border: rgba(56, 189, 248, 0.22);
  --speed-border-strong: rgba(56, 189, 248, 0.4);
  --speed-text: #e2e8f0;
  --speed-dim: rgba(148, 163, 184, 0.88);
  --speed-accent: #38bdf8;
  --speed-green: #34d399;
  --speed-yellow: #facc15;
  --speed-red: #fb7185;
  --speed-panel-glow: 0 10px 30px rgba(2, 6, 23, 0.45), inset 0 1px 0 rgba(255, 255, 255, 0.03);
}

.speed-panel {
  width: 100%;
  height: 100%;
  box-sizing: border-box;
  padding: 14px 16px 12px;
  color: var(--speed-text);
  background: var(--speed-bg);
  font-family: 'Outfit', system-ui, sans-serif;
  display: flex;
  flex-direction: column;
  border: 1px solid var(--speed-border);
  box-shadow: var(--speed-panel-glow);
  overflow: hidden;
}

.speed-panel-top {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  gap: 12px;
}

.speed-kicker {
  font-size: 10px;
  letter-spacing: 1.8px;
  text-transform: uppercase;
  color: var(--speed-accent);
  margin-bottom: 6px;
}

.speed-readout {
  display: flex;
  align-items: baseline;
  gap: 8px;
  line-height: 1;
}

.speed-value {
  font-size: clamp(30px, 5vw, 44px);
  font-weight: 700;
  letter-spacing: -1px;
}

.speed-unit {
  font-size: 14px;
  color: var(--speed-dim);
  text-transform: uppercase;
  letter-spacing: 1px;
}

.speed-secondary {
  margin-top: 6px;
  font-family: 'JetBrains Mono', monospace;
  font-size: 11px;
  color: var(--speed-dim);
}

.speed-status {
  padding: 6px 10px;
  border-radius: 999px;
  border: 1px solid var(--speed-border);
  font-family: 'JetBrains Mono', monospace;
  font-size: 10px;
  letter-spacing: 1px;
  text-transform: uppercase;
  white-space: nowrap;
}

.speed-status.stopped {
  color: var(--speed-dim);
}

.speed-status.creeping {
  color: var(--speed-green);
  border-color: rgba(52, 211, 153, 0.3);
}

.speed-status.nominal {
  color: var(--speed-accent);
  border-color: var(--speed-border-strong);
}

.speed-status.high {
  color: var(--speed-yellow);
  border-color: rgba(250, 204, 21, 0.35);
}

.speed-status.limit {
  color: var(--speed-red);
  border-color: rgba(251, 113, 133, 0.4);
}

.speed-scale {
  margin-top: 16px;
}

.speed-scale-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
  font-family: 'JetBrains Mono', monospace;
  font-size: 10px;
  color: var(--speed-dim);
  text-transform: uppercase;
  letter-spacing: 1px;
}

.speed-track {
  position: relative;
  height: 16px;
  border-radius: 999px;
  background:
    linear-gradient(90deg, rgba(52, 211, 153, 0.12) 0%, rgba(250, 204, 21, 0.12) 55%, rgba(251, 113, 133, 0.18) 100%),
    rgba(15, 23, 42, 0.95);
  border: 1px solid rgba(148, 163, 184, 0.12);
  overflow: hidden;
}

.speed-fill {
  position: absolute;
  inset: 0 auto 0 0;
  width: 0%;
  background: linear-gradient(90deg, #34d399 0%, #38bdf8 55%, #fb7185 100%);
  box-shadow: 0 0 16px rgba(56, 189, 248, 0.25);
}

.speed-command-marker {
  position: absolute;
  top: -4px;
  bottom: -4px;
  width: 2px;
  border-radius: 999px;
  background: rgba(255, 255, 255, 0.88);
  box-shadow: 0 0 0 1px rgba(2, 6, 23, 0.65);
}

.speed-ticks {
  position: relative;
  display: grid;
  grid-template-columns: repeat(5, 1fr);
  gap: 0;
  margin-top: 8px;
}

.speed-tick {
  display: flex;
  flex-direction: column;
  gap: 5px;
  align-items: center;
  font-family: 'JetBrains Mono', monospace;
  font-size: 10px;
  color: var(--speed-dim);
}

.speed-tick::before {
  content: "";
  width: 1px;
  height: 8px;
  background: rgba(148, 163, 184, 0.25);
}

.speed-metrics {
  margin-top: 16px;
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 8px;
}

.speed-metric {
  padding: 10px 12px;
  border-radius: 10px;
  border: 1px solid rgba(148, 163, 184, 0.12);
  background: rgba(9, 16, 28, 0.7);
}

.speed-metric-label {
  font-size: 10px;
  letter-spacing: 1.5px;
  text-transform: uppercase;
  color: var(--speed-dim);
  margin-bottom: 6px;
}

.speed-metric-value {
  font-family: 'JetBrains Mono', monospace;
  font-size: 15px;
  color: var(--speed-text);
}

.speed-footer {
  margin-top: auto;
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 10px;
  padding-top: 12px;
  font-family: 'JetBrains Mono', monospace;
  font-size: 10px;
  color: rgba(148, 163, 184, 0.72);
  text-transform: uppercase;
  letter-spacing: 1px;
}

.speed-footer strong {
  color: var(--speed-text);
  font-weight: 500;
}
`;

type DisplayUnit = "mps" | "kmh";

interface SpeedPanelSettings {
  odomTopic: string;
  cmdVelTopic: string;
  maxSpeedMps: number;
  warningSpeedMps: number;
  unit: DisplayUnit;
}

const DEFAULT_SETTINGS: SpeedPanelSettings = {
  odomTopic: "/odom",
  cmdVelTopic: "/cmd_vel",
  maxSpeedMps: 1,
  warningSpeedMps: 0.75,
  unit: "mps",
};

function buildSettingsTree(settings: SpeedPanelSettings): SettingsTreeNodes {
  return {
    general: {
      label: "General",
      fields: {
        odomTopic: {
          label: "Odom Topic",
          input: "string",
          value: settings.odomTopic,
        },
        cmdVelTopic: {
          label: "CmdVel Topic",
          input: "string",
          value: settings.cmdVelTopic,
        },
        unit: {
          label: "Primary Unit",
          input: "select",
          value: settings.unit,
          options: [
            { label: "m/s", value: "mps" },
            { label: "km/h", value: "kmh" },
          ],
        },
      },
    },
    limits: {
      label: "Limits",
      fields: {
        maxSpeedMps: {
          label: "Max Speed (m/s)",
          input: "number",
          value: settings.maxSpeedMps,
          min: 0.1,
          max: 5,
          step: 0.05,
        },
        warningSpeedMps: {
          label: "Warning Speed (m/s)",
          input: "number",
          value: settings.warningSpeedMps,
          min: 0.05,
          max: 5,
          step: 0.05,
        },
      },
    },
  };
}

function createTextEl(className: string, text = ""): HTMLDivElement {
  const el = document.createElement("div");
  el.className = className;
  el.textContent = text;
  return el;
}

function convertSpeed(valueMps: number, unit: DisplayUnit): number {
  return unit === "kmh" ? valueMps * 3.6 : valueMps;
}

function unitLabel(unit: DisplayUnit): string {
  return unit === "kmh" ? "km/h" : "m/s";
}

function formatSigned(value: number, digits = 2): string {
  const prefix = value > 0 ? "+" : "";
  return `${prefix}${value.toFixed(digits)}`;
}

function clamp01(value: number): number {
  return Math.max(0, Math.min(1, value));
}

function extractTwist(msg: Record<string, unknown>): { linearX?: number; angularZ?: number } {
  const twistContainer = (msg.twist as Record<string, unknown> | undefined) ?? msg;
  const twist =
    (twistContainer.twist as Record<string, unknown> | undefined) ?? twistContainer;
  const linear = twist.linear as Record<string, unknown> | undefined;
  const angular = twist.angular as Record<string, unknown> | undefined;

  const linearX = Number(linear?.x);
  const angularZ = Number(angular?.z);

  return {
    linearX: Number.isFinite(linearX) ? linearX : undefined,
    angularZ: Number.isFinite(angularZ) ? angularZ : undefined,
  };
}

function resolveStatus(speedAbsMps: number, settings: SpeedPanelSettings): { label: string; className: string } {
  if (speedAbsMps < 0.05) {
    return { label: "Stopped", className: "stopped" };
  }
  if (speedAbsMps < 0.2) {
    return { label: "Creeping", className: "creeping" };
  }
  if (speedAbsMps >= settings.maxSpeedMps) {
    return { label: "At Limit", className: "limit" };
  }
  if (speedAbsMps >= settings.warningSpeedMps) {
    return { label: "High", className: "high" };
  }
  return { label: "Nominal", className: "nominal" };
}

export function initSpeedPanel(context: PanelExtensionContext): () => void {
  const panelEl = context.panelElement;
  while (panelEl.firstChild) {
    panelEl.removeChild(panelEl.firstChild);
  }

  panelEl.style.width = "100%";
  panelEl.style.height = "100%";
  panelEl.style.overflow = "hidden";
  panelEl.style.background = "#060b13";

  const style = document.createElement("style");
  style.textContent = SPEED_PANEL_CSS;
  panelEl.appendChild(style);

  const root = document.createElement("div");
  root.className = "speed-panel";
  panelEl.appendChild(root);

  const top = document.createElement("div");
  top.className = "speed-panel-top";
  root.appendChild(top);

  const topLeft = document.createElement("div");
  top.appendChild(topLeft);

  const kicker = createTextEl("speed-kicker", "Drive Speed");
  topLeft.appendChild(kicker);

  const readout = document.createElement("div");
  readout.className = "speed-readout";
  topLeft.appendChild(readout);

  const speedValueEl = createTextEl("speed-value", "0.00");
  readout.appendChild(speedValueEl);

  const speedUnitEl = createTextEl("speed-unit", "m/s");
  readout.appendChild(speedUnitEl);

  const secondaryEl = createTextEl("speed-secondary", "0.00 km/h");
  topLeft.appendChild(secondaryEl);

  const statusEl = createTextEl("speed-status stopped", "Stopped");
  top.appendChild(statusEl);

  const scale = document.createElement("div");
  scale.className = "speed-scale";
  root.appendChild(scale);

  const scaleHeader = document.createElement("div");
  scaleHeader.className = "speed-scale-header";
  scale.appendChild(scaleHeader);

  const scaleHeaderLeft = createTextEl("speed-scale-note", "Actual vs Command");
  scaleHeader.appendChild(scaleHeaderLeft);

  const scaleHeaderRight = createTextEl("speed-scale-note", "Command marker");
  scaleHeader.appendChild(scaleHeaderRight);

  const track = document.createElement("div");
  track.className = "speed-track";
  scale.appendChild(track);

  const fill = document.createElement("div");
  fill.className = "speed-fill";
  track.appendChild(fill);

  const commandMarker = document.createElement("div");
  commandMarker.className = "speed-command-marker";
  track.appendChild(commandMarker);

  const ticks = document.createElement("div");
  ticks.className = "speed-ticks";
  scale.appendChild(ticks);

  const metrics = document.createElement("div");
  metrics.className = "speed-metrics";
  root.appendChild(metrics);

  const cmdMetric = document.createElement("div");
  cmdMetric.className = "speed-metric";
  metrics.appendChild(cmdMetric);
  cmdMetric.appendChild(createTextEl("speed-metric-label", "Commanded"));
  const cmdMetricValue = createTextEl("speed-metric-value", "0.00 m/s");
  cmdMetric.appendChild(cmdMetricValue);

  const deltaMetric = document.createElement("div");
  deltaMetric.className = "speed-metric";
  metrics.appendChild(deltaMetric);
  deltaMetric.appendChild(createTextEl("speed-metric-label", "Tracking Error"));
  const deltaMetricValue = createTextEl("speed-metric-value", "+0.00 m/s");
  deltaMetric.appendChild(deltaMetricValue);

  const yawMetric = document.createElement("div");
  yawMetric.className = "speed-metric";
  metrics.appendChild(yawMetric);
  yawMetric.appendChild(createTextEl("speed-metric-label", "Turn Rate"));
  const yawMetricValue = createTextEl("speed-metric-value", "0.00 rad/s");
  yawMetric.appendChild(yawMetricValue);

  const accelMetric = document.createElement("div");
  accelMetric.className = "speed-metric";
  metrics.appendChild(accelMetric);
  accelMetric.appendChild(createTextEl("speed-metric-label", "Acceleration"));
  const accelMetricValue = createTextEl("speed-metric-value", "0.00 m/s²");
  accelMetric.appendChild(accelMetricValue);

  const footer = document.createElement("div");
  footer.className = "speed-footer";
  root.appendChild(footer);

  const footerLeft = document.createElement("div");
  footer.appendChild(footerLeft);

  const footerRight = document.createElement("div");
  footer.appendChild(footerRight);

  let settings: SpeedPanelSettings = { ...DEFAULT_SETTINGS };
  const savedState = context.initialState as (Partial<SpeedPanelSettings> & { foxglovePanelTitle?: string }) | undefined;
  const foxglovePanelTitle =
    typeof savedState?.foxglovePanelTitle === "string" ? savedState.foxglovePanelTitle : "Speed";
  if (savedState) {
    settings = { ...DEFAULT_SETTINGS, ...savedState };
  }

  let actualLinearX = 0;
  let actualAngularZ = 0;
  let commandedLinearX = 0;
  let lastActualSpeed = 0;
  let acceleration = 0;
  let lastActualTimeMs = 0;

  function saveState() {
    context.saveState({ ...settings, foxglovePanelTitle });
  }

  function updateTicks() {
    ticks.textContent = "";
    for (let i = 0; i < 5; i++) {
      const tick = document.createElement("div");
      tick.className = "speed-tick";
      const tickValue = (settings.maxSpeedMps * i) / 4;
      tick.textContent = convertSpeed(tickValue, settings.unit).toFixed(i === 0 ? 0 : 2);
      ticks.appendChild(tick);
    }
  }

  function updateDisplay() {
    const displayUnit = settings.unit;
    const actualAbs = Math.abs(actualLinearX);
    const commandAbs = Math.abs(commandedLinearX);
    const convertedActual = convertSpeed(actualLinearX, displayUnit);
    const secondaryUnit = displayUnit === "mps" ? "kmh" : "mps";
    const status = resolveStatus(actualAbs, settings);

    speedValueEl.textContent = convertedActual.toFixed(Math.abs(convertedActual) >= 1 ? 1 : 2);
    speedUnitEl.textContent = unitLabel(displayUnit);
    secondaryEl.textContent = `${convertSpeed(actualLinearX, secondaryUnit).toFixed(2)} ${unitLabel(secondaryUnit)}`;

    statusEl.className = `speed-status ${status.className}`;
    statusEl.textContent = status.label;

    fill.style.width = `${clamp01(actualAbs / Math.max(settings.maxSpeedMps, 0.001)) * 100}%`;
    commandMarker.style.left = `calc(${clamp01(commandAbs / Math.max(settings.maxSpeedMps, 0.001)) * 100}% - 1px)`;

    cmdMetricValue.textContent = `${convertSpeed(commandedLinearX, displayUnit).toFixed(2)} ${unitLabel(displayUnit)}`;
    deltaMetricValue.textContent = `${formatSigned(convertSpeed(actualLinearX - commandedLinearX, displayUnit), 2)} ${unitLabel(displayUnit)}`;
    yawMetricValue.textContent = `${actualAngularZ.toFixed(2)} rad/s`;
    accelMetricValue.textContent = `${formatSigned(acceleration, 2)} m/s²`;

    footerLeft.innerHTML = `Scale <strong>0-${settings.maxSpeedMps.toFixed(2)} m/s</strong>`;
    footerRight.innerHTML = `Warn <strong>${settings.warningSpeedMps.toFixed(2)} m/s</strong>`;
  }

  function resubscribe() {
    const topics: string[] = [];
    if (settings.odomTopic) topics.push(settings.odomTopic);
    if (settings.cmdVelTopic) topics.push(settings.cmdVelTopic);
    context.subscribe(topics);
  }

  function handleSettingsAction(action: SettingsTreeAction) {
    if (action.action !== "update" || !action.payload) {
      return;
    }
    const field = action.payload.path[action.payload.path.length - 1] as keyof SpeedPanelSettings | undefined;
    if (!field || !(field in settings)) {
      return;
    }

    (settings as unknown as Record<string, unknown>)[field] = action.payload.value;

    if (field === "odomTopic" || field === "cmdVelTopic") {
      resubscribe();
    }

    updateTicks();
    updateDisplay();
    context.updatePanelSettingsEditor({
      actionHandler: handleSettingsAction,
      nodes: buildSettingsTree(settings),
    });
    saveState();
  }

  context.updatePanelSettingsEditor({
    actionHandler: handleSettingsAction,
    nodes: buildSettingsTree(settings),
  });

  updateTicks();
  updateDisplay();
  resubscribe();

  context.onRender = (renderState: Immutable<RenderState>, done: () => void) => {
    if (renderState.currentFrame) {
      for (const msg of renderState.currentFrame) {
        if (msg.topic === settings.odomTopic) {
          const twist = extractTwist(msg.message as Record<string, unknown>);
          if (twist.linearX != null) {
            const nowMs = Date.now();
            if (lastActualTimeMs > 0) {
              const dt = (nowMs - lastActualTimeMs) / 1000;
              if (dt > 0.02) {
                acceleration = (twist.linearX - lastActualSpeed) / dt;
              }
            }
            lastActualTimeMs = nowMs;
            lastActualSpeed = twist.linearX;
            actualLinearX = twist.linearX;
          }
          if (twist.angularZ != null) {
            actualAngularZ = twist.angularZ;
          }
        } else if (msg.topic === settings.cmdVelTopic) {
          const twist = extractTwist(msg.message as Record<string, unknown>);
          if (twist.linearX != null) {
            commandedLinearX = twist.linearX;
          }
        }
      }
    }

    updateDisplay();
    done();
  };

  context.watch("currentFrame");

  return () => {
    while (panelEl.firstChild) {
      panelEl.removeChild(panelEl.firstChild);
    }
  };
}
