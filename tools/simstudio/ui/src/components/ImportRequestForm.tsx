import type { ImportKind, JsonObject } from "../api.ts";
import { setImportRequestField } from "../importDraft.ts";

type FieldPath = readonly (string | number)[];

interface ImportRequestFormProps {
  kind: ImportKind;
  request: JsonObject;
  disabled?: boolean;
  onChange: (request: JsonObject) => void;
}

interface FieldProps {
  label: string;
  value: unknown;
  disabled?: boolean;
  onValue: (value: unknown) => void;
}

function fieldValue(request: JsonObject, path: FieldPath): unknown {
  let cursor: unknown = request;
  for (const segment of path) {
    if (typeof segment === "number") {
      if (!Array.isArray(cursor)) return undefined;
      cursor = cursor[segment];
    } else {
      if (cursor === null || typeof cursor !== "object" || Array.isArray(cursor)) return undefined;
      cursor = (cursor as JsonObject)[segment];
    }
  }
  return cursor;
}

function GuidedTextField({ label, value, disabled, onValue }: FieldProps) {
  return (
    <label className="field">
      <span>{label}</span>
      <input
        type="text"
        disabled={disabled}
        value={typeof value === "string" ? value : ""}
        onChange={(event) => onValue(event.target.value)}
      />
    </label>
  );
}

function GuidedNullableField({ label, value, disabled, onValue }: FieldProps) {
  return (
    <label className="field">
      <span>{label}</span>
      <input
        type="text"
        disabled={disabled}
        value={typeof value === "string" ? value : ""}
        placeholder="none"
        onChange={(event) => onValue(event.target.value || null)}
      />
    </label>
  );
}

function GuidedNumberField({ label, value, disabled, onValue }: FieldProps) {
  return (
    <label className="field">
      <span>{label}</span>
      <input
        type="number"
        step="any"
        disabled={disabled}
        value={typeof value === "number" && Number.isFinite(value) ? value : ""}
        onChange={(event) => {
          const next = Number(event.target.value);
          if (event.target.value !== "" && Number.isFinite(next)) onValue(next);
        }}
      />
    </label>
  );
}

export function ImportRequestForm({ kind, request, disabled, onChange }: ImportRequestFormProps) {
  const set = (path: FieldPath, value: unknown) => {
    onChange(setImportRequestField(request, path, value));
  };
  const text = (label: string, path: FieldPath) => (
    <GuidedTextField
      label={label}
      value={fieldValue(request, path)}
      disabled={disabled}
      onValue={(value) => set(path, value)}
    />
  );
  const nullable = (label: string, path: FieldPath) => (
    <GuidedNullableField
      label={label}
      value={fieldValue(request, path)}
      disabled={disabled}
      onValue={(value) => set(path, value)}
    />
  );
  const number = (label: string, path: FieldPath) => (
    <GuidedNumberField
      label={label}
      value={fieldValue(request, path)}
      disabled={disabled}
      onValue={(value) => set(path, value)}
    />
  );
  const identityRoot: FieldPath = kind === "robot" ? [] : ["package"];
  const provenanceRoot: FieldPath = kind === "robot" ? ["provenance"] : ["source", "provenance"];
  const at = (root: FieldPath, ...tail: (string | number)[]) => [...root, ...tail] as FieldPath;

  return (
    <div className="guided-import-editor">
      <section>
        <h3>Package identity</h3>
        <div className="form-grid form-grid--three">
          {text("Package ID", at(identityRoot, "id"))}
          {text("Version", at(identityRoot, "version"))}
          {text("Description", at(identityRoot, "description"))}
        </div>
      </section>

      <section>
        <h3>Provenance and license</h3>
        <div className="form-grid form-grid--two">
          {text("Owner", at(provenanceRoot, "owner"))}
          {text("License", at(provenanceRoot, "license"))}
          {text("License file", at(provenanceRoot, "license_file"))}
          {text("Source URI", at(provenanceRoot, "source_uri"))}
        </div>
      </section>

      {kind === "robot" ? (
        <>
          <section>
            <h3>Robot model</h3>
            <div className="form-grid form-grid--two">
              <label className="field">
                <span>Source format</span>
                <select
                  disabled={disabled}
                  value={String(fieldValue(request, ["source_format"]) ?? "mjcf")}
                  onChange={(event) => set(["source_format"], event.target.value)}
                >
                  <option value="mjcf">MJCF</option>
                  <option value="urdf">URDF</option>
                </select>
              </label>
              {text("Source model", ["source_model"])}
              {text("Attach root", ["physics", "attach_root"])}
              {text("Root joint", ["physics", "root_joint"])}
              {text("Visual binding", ["visual", "binding"])}
              {text("Semantic class", ["semantic", "class"])}
            </div>
          </section>
          <section>
            <h3>Defaults</h3>
            <div className="form-grid form-grid--two">
              {nullable("Controller package", ["defaults", "controller"])}
              {nullable("Sensor rig package", ["defaults", "sensor_rig"])}
            </div>
          </section>
        </>
      ) : (
        <>
          <section>
            <h3>Terrain source</h3>
            <div className="form-grid form-grid--three">
              {text("Heightmap", ["heightmap", "path"])}
              {number("Width (px)", ["heightmap", "width"])}
              {number("Height (px)", ["heightmap", "height"])}
              {number("Extent X (m)", ["heightmap", "extent_m", 0])}
              {number("Extent Y (m)", ["heightmap", "extent_m", 1])}
              {number("Elevation min (m)", ["heightmap", "elevation_min_m"])}
              {number("Elevation max (m)", ["heightmap", "elevation_max_m"])}
              <label className="field">
                <span>R16 endian</span>
                <select
                  disabled={disabled}
                  value={String(fieldValue(request, ["heightmap", "endian"]) ?? "little")}
                  onChange={(event) => set(["heightmap", "endian"], event.target.value)}
                >
                  <option value="little">little</option>
                  <option value="big">big</option>
                </select>
              </label>
            </div>
          </section>
          <section>
            <h3>Unreal projection and spawn</h3>
            <div className="form-grid form-grid--two">
              {text("Visual binding", ["visual", "binding"])}
              {text("UE Level", ["visual", "level"])}
            </div>
            <div className="form-grid form-grid--three guided-spawn-grid">
              {number("Spawn X (m)", ["spawn", "position_m", 0])}
              {number("Spawn Y (m)", ["spawn", "position_m", 1])}
              {number("Spawn Z (m)", ["spawn", "position_m", 2])}
              {number("Height tolerance (m)", ["spawn", "height_tolerance_m"])}
            </div>
          </section>
        </>
      )}
    </div>
  );
}
