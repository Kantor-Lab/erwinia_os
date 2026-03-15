# Erwinia Map Panel

Foxglove extension panel that displays an interactive satellite/street map with GPS robot tracking and color-coded grid overlay for precision agriculture.

## Features

- Satellite and street map tiles (Esri World Imagery / OpenStreetMap)
- Real-time GPS robot position from `sensor_msgs/NavSatFix`
- Color-coded grid overlay from custom heatmap topic
- Robot trail visualization
- Configurable color scales (Red-Yellow-Green, Viridis, Thermal)
- Settings panel for topic selection, map style, opacity, and auto-centering

## Usage

1. Add the "ErwiniaMap" panel to your Foxglove layout
2. Configure the GPS topic (default: `/nav_sat_fix`)
3. Configure the grid topic (default: `/field_heatmap`)
4. The map will auto-center on the first GPS fix

## Grid Message Format

The grid topic expects messages with a `cells` or `data` array:

```json
{
  "cells": [
    { "lat": 38.123, "lon": -121.456, "value": 0.8, "size": 0.00005 },
    { "lat": 38.124, "lon": -121.457, "value": 0.3 }
  ]
}
```

- `value`: 0-1 normalized value mapped to the selected color scale
- `size`: optional cell size in degrees (default ~5m)
