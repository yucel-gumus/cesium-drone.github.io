// Reuse the unmarked belly finish only on the branded part of the top shell.
// This edits UV geometry, not the embedded colour/normal/ORM images. Splitting
// triangles at the patch boundary keeps paint, seams and all other parts intact.
module.exports = function cleanDroneModel(source) {
  const jsonLength = source.readUInt32LE(12);
  const gltf = JSON.parse(source.subarray(20, 20 + jsonLength).toString());
  const binStart = 28 + jsonLength;
  const binary = source.subarray(binStart, binStart + source.readUInt32LE(20 + jsonLength));
  const body = gltf.nodes.find((node) => node.name === "Body_low");
  if (!body) throw new Error("Drone body mesh missing");
  const primitive = gltf.meshes[body.mesh].primitives[0];
  const fields = ["POSITION", "NORMAL", "TANGENT", "TEXCOORD_0"];
  const widths = [3, 3, 4, 2];
  function read(index, width) {
    const accessor = gltf.accessors[index];
    const view = gltf.bufferViews[accessor.bufferView];
    const bytes = { 5123: 2, 5125: 4, 5126: 4 }[accessor.componentType];
    const method = { 5123: "readUInt16LE", 5125: "readUInt32LE", 5126: "readFloatLE" }[accessor.componentType];
    if (!method) throw new Error("Unsupported drone accessor");
    return Array.from({ length: accessor.count }, (_, i) =>
      Array.from({ length: width }, (_, j) => binary[method](
        (view.byteOffset || 0) + (accessor.byteOffset || 0) +
        i * (view.byteStride || width * bytes) + j * bytes,
      )),
    );
  }
  const attributes = fields.map((field, i) => read(primitive.attributes[field], widths[i]));
  const vertices = attributes[0].map((_, i) => attributes.flatMap((values) => values[i]));
  const indices = read(primitive.indices, 1).flat();
  // UV bounds of the lettering, symbol and their baked relief, with padding.
  const planes = [[10, .208, 1], [10, .263, -1], [11, .18, 1], [11, .407, -1]];
  const cleanOffset = -.142755;
  function clip(polygon, axis, edge, sign) {
    const result = [];
    for (let i = 0; i < polygon.length; i++) {
      const a = polygon[i], b = polygon[(i + 1) % polygon.length];
      const da = (a[axis] - edge) * sign, db = (b[axis] - edge) * sign;
      if (da >= 0) result.push(a);
      if ((da < 0 && db > 0) || (da > 0 && db < 0)) {
        const t = da / (da - db);
        result.push(a.map((value, j) => value + (b[j] - value) * t));
      }
    }
    return result;
  }
  const output = [];
  let patchedTriangles = 0;
  function emit(polygon, patched) {
    for (let i = 1; i < polygon.length - 1; i++) {
      const triangle = [polygon[0], polygon[i], polygon[i + 1]];
      const a = triangle[0], b = triangle[1], c = triangle[2];
      const area = Math.hypot(
        (b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]),
        (b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]),
        (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]),
      );
      if (area < 1e-12) continue;
      for (const vertex of triangle) {
        const copy = vertex.slice();
        if (patched) copy[10] += cleanOffset;
        // Clipped vertices interpolate the original tangent frame. Restore its
        // unit lengths and orthogonality required by glTF before serialization.
        const normalLength = Math.hypot(...copy.slice(3, 6));
        for (let j = 3; j < 6; j++) copy[j] /= normalLength;
        const dot = copy[3] * copy[6] + copy[4] * copy[7] + copy[5] * copy[8];
        for (let j = 0; j < 3; j++) copy[6 + j] -= dot * copy[3 + j];
        const tangentLength = Math.hypot(...copy.slice(6, 9));
        for (let j = 6; j < 9; j++) copy[j] /= tangentLength;
        copy[9] = copy[9] < 0 ? -1 : 1;
        output.push(copy);
      }
      if (patched) patchedTriangles++;
    }
  }
  for (let i = 0; i < indices.length; i += 3) {
    let inside = indices.slice(i, i + 3).map((index) => vertices[index]);
    for (const [axis, edge, sign] of planes) {
      emit(clip(inside, axis, edge, -sign), false);
      inside = clip(inside, axis, edge, sign);
      if (!inside.length) break;
    }
    emit(inside, true);
  }
  if (!patchedTriangles) throw new Error("Drone branding UV patch did not intersect the shell");
  const chunks = [binary];
  let byteLength = binary.length;
  function append(buffer, accessor, target) {
    const padding = (4 - byteLength % 4) % 4;
    chunks.push(Buffer.alloc(padding), buffer);
    byteLength += padding;
    const bufferView = gltf.bufferViews.push({ buffer: 0, byteOffset: byteLength, byteLength: buffer.length, target }) - 1;
    byteLength += buffer.length;
    return gltf.accessors.push({ ...accessor, bufferView }) - 1;
  }
  let offset = 0;
  fields.forEach((field, k) => {
    const width = widths[k], buffer = Buffer.alloc(output.length * width * 4);
    output.forEach((vertex, i) => {
      for (let j = 0; j < width; j++) buffer.writeFloatLE(vertex[offset + j], (i * width + j) * 4);
    });
    const accessor = { componentType: 5126, count: output.length, type: `VEC${width}` };
    if (field === "POSITION") {
      accessor.min = [0, 1, 2].map((j) => Math.min(...output.map((v) => v[j])));
      accessor.max = [0, 1, 2].map((j) => Math.max(...output.map((v) => v[j])));
    }
    primitive.attributes[field] = append(buffer, accessor, 34962);
    offset += width;
  });
  const indexBuffer = Buffer.alloc(output.length * 4);
  output.forEach((_, i) => indexBuffer.writeUInt32LE(i, i * 4));
  primitive.indices = append(indexBuffer, { componentType: 5125, count: output.length, type: "SCALAR" }, 34963);
  gltf.buffers[0].byteLength = byteLength;
  const json = Buffer.from(JSON.stringify(gltf));
  const jsonPad = Buffer.alloc((4 - json.length % 4) % 4, 0x20);
  const binPad = Buffer.alloc((4 - byteLength % 4) % 4);
  const header = Buffer.alloc(20), binHeader = Buffer.alloc(8);
  header.writeUInt32LE(0x46546c67, 0);
  header.writeUInt32LE(2, 4);
  header.writeUInt32LE(28 + json.length + jsonPad.length + byteLength + binPad.length, 8);
  header.writeUInt32LE(json.length + jsonPad.length, 12);
  header.writeUInt32LE(0x4e4f534a, 16);
  binHeader.writeUInt32LE(byteLength + binPad.length, 0);
  binHeader.writeUInt32LE(0x004e4942, 4);
  return Buffer.concat([header, json, jsonPad, binHeader, ...chunks, binPad]);
};
