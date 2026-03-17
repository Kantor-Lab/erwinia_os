import { ExtensionContext } from "@foxglove/extension";
import { initMapPanel } from "./MapPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "ErwiniaMap",
    initPanel: initMapPanel,
  });
}
