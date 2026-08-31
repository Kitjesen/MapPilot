export function formatBytes(bytes: number): string {
  if (!Number.isFinite(bytes) || bytes < 0) return "未知";
  if (bytes < 1024) return `${bytes} B`;
  const units = ["KB", "MB", "GB"];
  let value = bytes / 1024;
  let unit = units[0];
  for (let index = 1; index < units.length && value >= 1024; index += 1) {
    value /= 1024;
    unit = units[index];
  }
  return `${value.toFixed(value >= 10 ? 1 : 2)} ${unit}`;
}

export function shortDigest(value: unknown, length = 12): string {
  return typeof value === "string" && value.length > length
    ? `${value.slice(0, length)}…`
    : typeof value === "string"
      ? value
      : "未提供";
}

export function formatDate(value: unknown): string {
  if (typeof value !== "string" || !value) return "未知";
  const date = new Date(value);
  return Number.isNaN(date.valueOf()) ? value : date.toLocaleString("zh-CN");
}

export function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : "发生未知错误";
}

export function operationKey(scope: string): string {
  return `${scope}-${globalThis.crypto.randomUUID()}`;
}

export function stringField(value: unknown, fallback = "未提供"): string {
  return typeof value === "string" && value ? value : fallback;
}
