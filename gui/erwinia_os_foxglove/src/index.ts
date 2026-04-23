import { ExtensionContext } from "@foxglove/extension";
import { initMapPanel } from "./MapPanel";
import { initSpeedPanel } from "./SpeedPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "ErwiniaMap",
    initPanel: initMapPanel,
  });
  extensionContext.registerPanel({
    name: "ErwiniaSpeed",
    initPanel: initSpeedPanel,
  });
}
