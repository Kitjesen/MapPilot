import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import App from "./App.tsx";
import "./design-system/index.css";
import "./styles.css";
import "./lightfield.css";

const root = document.getElementById("root");

if (!root) {
  throw new Error("SimStudio root element was not found");
}

createRoot(root).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
