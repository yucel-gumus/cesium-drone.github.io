import http from "node:http";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
const root = fileURLToPath(new URL("../dist/", import.meta.url));
const prefix = "/cesium-drone.github.io/";
const types = {
  ".html": "text/html",
  ".js": "text/javascript",
  ".css": "text/css",
  ".json": "application/json",
  ".glb": "model/gltf-binary",
  ".wasm": "application/wasm",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".svg": "image/svg+xml",
};
http
  .createServer((request, response) => {
    let pathname;
    try {
      pathname = decodeURIComponent(
        new URL(request.url, "http://localhost").pathname,
      );
    } catch {
      response.writeHead(400).end();
      return;
    }
    if (pathname === "/cesium-drone.github.io") {
      response.writeHead(302, { Location: prefix }).end();
      return;
    }
    if (pathname.startsWith(prefix))
      pathname = pathname.slice(prefix.length) || "/";
    const file = path.resolve(
      root,
      `.${pathname.startsWith("/") ? pathname : `/${pathname}`}`,
      pathname.endsWith("/") ? "index.html" : "",
    );
    if (
      !file.startsWith(root) ||
      !fs.existsSync(file) ||
      !fs.statSync(file).isFile()
    ) {
      response.writeHead(404).end();
      return;
    }
    response.writeHead(200, {
      "Content-Type": types[path.extname(file)] || "application/octet-stream",
      "Cache-Control": "no-store",
    });
    fs.createReadStream(file).pipe(response);
  })
  .listen(9001, "127.0.0.1", () =>
    console.log("Preview: http://127.0.0.1:9001/cesium-drone.github.io/"),
  );
