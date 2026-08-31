import {
  AlertTriangle,
  Inbox,
  RefreshCw,
} from "lucide-react";
import type { ReactNode } from "react";

export function ErrorPanel({
  title,
  message,
  onRetry,
}: {
  title: string;
  message: string;
  onRetry?: () => void;
}) {
  return (
    <div className="state-panel state-panel--error" role="alert">
      <AlertTriangle aria-hidden="true" size={20} strokeWidth={1.7} />
      <div>
        <strong>{title}</strong>
        <p>{message}</p>
      </div>
      {onRetry ? (
        <button className="button button--quiet" type="button" onClick={onRetry}>
          <RefreshCw aria-hidden="true" size={16} strokeWidth={1.7} />
          重试
        </button>
      ) : null}
    </div>
  );
}

export function EmptyPanel({
  title,
  children,
}: {
  title: string;
  children: ReactNode;
}) {
  return (
    <div className="state-panel state-panel--empty">
      <Inbox aria-hidden="true" size={24} strokeWidth={1.5} />
      <div>
        <strong>{title}</strong>
        <p>{children}</p>
      </div>
    </div>
  );
}

export function SkeletonRows({ count = 5 }: { count?: number }) {
  return (
    <div className="skeleton-list" aria-label="正在加载" aria-busy="true">
      {Array.from({ length: count }, (_, index) => (
        <div className="skeleton-row" key={index}>
          <span className="skeleton skeleton--title" />
          <span className="skeleton skeleton--copy" />
        </div>
      ))}
    </div>
  );
}

export function StatusBadge({ value }: { value: string }) {
  const normalized = value.toLowerCase();
  let tone = "neutral";
  if (
    ["ok", "ready", "running", "active", "qualified", "passed", "composed", "promoted"].includes(
      normalized,
    )
  ) {
    tone = "positive";
  } else if (["failed", "error", "invalid", "stale"].includes(normalized)) {
    tone = "danger";
  } else if (["paused", "unverified", "created", "preparing"].includes(normalized)) {
    tone = "warning";
  }
  return <span className={`status-badge status-badge--${tone}`}>{value}</span>;
}

export function SectionHeading({
  id,
  title,
  description,
  action,
}: {
  id: string;
  title: string;
  description: string;
  action?: ReactNode;
}) {
  return (
    <header className="section-heading">
      <div>
        <h1 id={id}>{title}</h1>
        <p>{description}</p>
      </div>
      {action ? <div className="section-heading__action">{action}</div> : null}
    </header>
  );
}
