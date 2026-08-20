import { createServer } from "node:http";
import { createReadStream } from "node:fs";
import { readFile, stat } from "node:fs/promises";
import { extname, resolve } from "node:path";

const root = resolve("frontend");
const port = Number(process.env.MANTA_PREVIEW_PORT || 4173);
const contentTypes = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".mp4": "video/mp4"
};

createServer(async (request, response) => {
    const requestPath = decodeURIComponent((request.url || "/").split("?")[0]);
    const filePath = resolve(root, `.${requestPath === "/" ? "/mobile-preview.html" : requestPath}`);

    if (!filePath.startsWith(root)) {
        response.writeHead(403);
        response.end("Forbidden");
        return;
    }

    try {
        const type = contentTypes[extname(filePath).toLowerCase()] || "application/octet-stream";
        const range = request.headers.range;
        if (range && type === "video/mp4") {
            const { size } = await stat(filePath);
            const match = /^bytes=(\d*)-(\d*)$/.exec(range);
            if (!match) {
                response.writeHead(416, { "Content-Range": `bytes */${size}` });
                response.end();
                return;
            }
            const start = match[1] ? Number(match[1]) : 0;
            const end = match[2] ? Math.min(Number(match[2]), size - 1) : size - 1;
            if (start > end || start >= size) {
                response.writeHead(416, { "Content-Range": `bytes */${size}` });
                response.end();
                return;
            }
            response.writeHead(206, {
                "Content-Type": type,
                "Content-Length": end - start + 1,
                "Content-Range": `bytes ${start}-${end}/${size}`,
                "Accept-Ranges": "bytes",
                "Cache-Control": "no-store"
            });
            createReadStream(filePath, { start, end }).pipe(response);
            return;
        }
        const body = await readFile(filePath);
        response.writeHead(200, {
            "Content-Type": type,
            "Accept-Ranges": type === "video/mp4" ? "bytes" : "none",
            "Cache-Control": "no-store"
        });
        response.end(body);
    } catch {
        response.writeHead(404);
        response.end("Not found");
    }
}).listen(port, "127.0.0.1", () => {
    process.stdout.write(`MANTA preview: http://127.0.0.1:${port}/mobile-preview.html\n`);
});
