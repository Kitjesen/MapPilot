import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

const target = process.env.SIMSTUDIO_API_ORIGIN ?? "http://127.0.0.1:8765";
const loopbackOrigin = /^http:\/\/(127\.0\.0\.1|localhost)(:\d{1,5})?$/;

if (!loopbackOrigin.test(target)) {
  throw new Error("SIMSTUDIO_API_ORIGIN must be a loopback HTTP origin");
}

export default defineConfig({
  plugins: [react()],
  server: {
    host: "127.0.0.1",
    port: 8766,
    strictPort: true,
    proxy: {
      "/api/sim/v1": {
        target,
        changeOrigin: false,
      },
    },
  },
  preview: {
    host: "127.0.0.1",
    port: 8766,
    strictPort: true,
  },
  build: {
    outDir: "dist",
    emptyOutDir: true,
  },
});
