import L from "leaflet";

const DB_NAME = "erwinia-tile-cache";
const STORE_NAME = "tiles";
const DB_VERSION = 1;

function openDB(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open(DB_NAME, DB_VERSION);
    req.onupgradeneeded = () => {
      const db = req.result;
      if (!db.objectStoreNames.contains(STORE_NAME)) {
        db.createObjectStore(STORE_NAME);
      }
    };
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error);
  });
}

function getCachedTile(db: IDBDatabase, key: string): Promise<Blob | undefined> {
  return new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_NAME, "readonly");
    const store = tx.objectStore(STORE_NAME);
    const req = store.get(key);
    req.onsuccess = () => resolve(req.result as Blob | undefined);
    req.onerror = () => reject(req.error);
  });
}

function cacheTile(db: IDBDatabase, key: string, blob: Blob): Promise<void> {
  return new Promise((resolve, reject) => {
    const tx = db.transaction(STORE_NAME, "readwrite");
    const store = tx.objectStore(STORE_NAME);
    const req = store.put(blob, key);
    req.onsuccess = () => resolve();
    req.onerror = () => reject(req.error);
  });
}

export class CachedTileLayer extends L.TileLayer {
  private db: IDBDatabase | undefined;
  private dbReady: Promise<IDBDatabase>;

  constructor(urlTemplate: string, options?: L.TileLayerOptions) {
    super(urlTemplate, options);
    this.dbReady = openDB().then((db) => {
      this.db = db;
      return db;
    });
  }

  createTile(coords: L.Coords, done: L.DoneCallback): HTMLElement {
    const tile = document.createElement("img");
    tile.alt = "";
    tile.setAttribute("role", "presentation");

    const url = this.getTileUrl(coords);
    const cacheKey = `${coords.z}/${coords.x}/${coords.y}`;

    this.dbReady.then(async (db) => {
      // Try cache first
      const cached = await getCachedTile(db, cacheKey);
      if (cached) {
        tile.src = URL.createObjectURL(cached);
        done(undefined, tile);
        return;
      }

      // Fetch from network
      try {
        const resp = await fetch(url);
        if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
        const blob = await resp.blob();
        await cacheTile(db, cacheKey, blob);
        tile.src = URL.createObjectURL(blob);
        done(undefined, tile);
      } catch (err) {
        // Offline and not cached — grey placeholder
        tile.style.background = "#2a2a3e";
        done(err as Error, tile);
      }
    });

    return tile;
  }
}

/** Pre-download tiles for a square area at zoom levels 15-18 */
export async function downloadTilesForArea(
  lat: number,
  lon: number,
  sizeKm: number,
  onProgress: (done: number, total: number) => void,
): Promise<void> {
  const db = await openDB();
  const halfDeg = (sizeKm / 2) / 111.32; // approx degrees
  const minLat = lat - halfDeg;
  const maxLat = lat + halfDeg;
  const minLon = lon - halfDeg;
  const maxLon = lon + halfDeg;

  // Collect all tile coords for zoom 15-18
  const tileCoords: { z: number; x: number; y: number }[] = [];
  for (let z = 15; z <= 18; z++) {
    const minTileX = lonToTileX(minLon, z);
    const maxTileX = lonToTileX(maxLon, z);
    const minTileY = latToTileY(maxLat, z); // note: y is inverted
    const maxTileY = latToTileY(minLat, z);
    for (let x = minTileX; x <= maxTileX; x++) {
      for (let y = minTileY; y <= maxTileY; y++) {
        tileCoords.push({ z, x, y });
      }
    }
  }

  let completed = 0;
  const total = tileCoords.length;
  onProgress(0, total);

  // Download in batches of 10 to avoid overwhelming the server
  const batchSize = 10;
  for (let i = 0; i < tileCoords.length; i += batchSize) {
    const batch = tileCoords.slice(i, i + batchSize);
    await Promise.all(
      batch.map(async ({ z, x, y }) => {
        const cacheKey = `${z}/${x}/${y}`;
        // Skip if already cached
        const existing = await getCachedTile(db, cacheKey);
        if (existing) {
          completed++;
          onProgress(completed, total);
          return;
        }

        // Use satellite tiles URL (Esri)
        const url = `https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${z}/${y}/${x}`;
        try {
          const resp = await fetch(url);
          if (resp.ok) {
            const blob = await resp.blob();
            await cacheTile(db, cacheKey, blob);
          }
        } catch {
          // Skip failed tiles silently
        }
        completed++;
        onProgress(completed, total);
      }),
    );
  }
}

function lonToTileX(lon: number, zoom: number): number {
  return Math.floor(((lon + 180) / 360) * Math.pow(2, zoom));
}

function latToTileY(lat: number, zoom: number): number {
  const latRad = (lat * Math.PI) / 180;
  return Math.floor(
    ((1 - Math.log(Math.tan(latRad) + 1 / Math.cos(latRad)) / Math.PI) / 2) *
      Math.pow(2, zoom),
  );
}
