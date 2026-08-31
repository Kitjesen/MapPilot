import {
  Boxes,
  FileCode2,
  GitBranch,
  RefreshCw,
  Search,
  ShieldCheck,
} from "lucide-react";
import { useEffect, useMemo, useState } from "react";

import type {
  CatalogList,
  PackageDetail,
  PackageSummary,
  SimStudioClient,
} from "../api.ts";
import { errorMessage } from "../format.ts";
import {
  EmptyPanel,
  ErrorPanel,
  SectionHeading,
  SkeletonRows,
  StatusBadge,
} from "./Common.tsx";

interface PackageLibraryProps {
  client: SimStudioClient;
  catalog: CatalogList | null;
  loading: boolean;
  error: string | null;
  onReload: () => void;
}

function packageKey(item: PackageSummary): string {
  return `${item.package.kind}:${item.package.ref}`;
}

export function PackageLibrary({
  client,
  catalog,
  loading,
  error,
  onReload,
}: PackageLibraryProps) {
  const [query, setQuery] = useState("");
  const [kind, setKind] = useState("all");
  const [selectedKey, setSelectedKey] = useState("");
  const [detail, setDetail] = useState<PackageDetail | null>(null);
  const [detailLoading, setDetailLoading] = useState(false);
  const [detailError, setDetailError] = useState<string | null>(null);

  const packages = useMemo(() => catalog?.packages ?? [], [catalog]);
  const kinds = useMemo(
    () => [...new Set(packages.map((item) => item.package.kind))].sort(),
    [packages],
  );
  const filtered = useMemo(() => {
    const needle = query.trim().toLowerCase();
    return packages.filter((item) => {
      if (kind !== "all" && item.package.kind !== kind) return false;
      if (!needle) return true;
      return [
        item.package.ref,
        item.package.kind,
        item.description ?? "",
      ].some((value) => value.toLowerCase().includes(needle));
    });
  }, [kind, packages, query]);

  const effectiveSelectedKey = filtered.some((item) => packageKey(item) === selectedKey)
    ? selectedKey
    : (filtered[0] ? packageKey(filtered[0]) : "");
  const selected = filtered.find((item) => packageKey(item) === effectiveSelectedKey) ?? null;

  useEffect(() => {
    let active = true;
    if (!selected) {
      return () => {
        active = false;
      };
    }
    void (async () => {
      await Promise.resolve();
      if (!active) return;
      setDetailLoading(true);
      setDetailError(null);
      try {
        const result = await client.inspectPackage(selected.package.kind, selected.package.ref);
        if (active) setDetail(result);
      } catch (reason: unknown) {
        if (active) setDetailError(errorMessage(reason));
      } finally {
        if (active) setDetailLoading(false);
      }
    })();
    return () => {
      active = false;
    };
  }, [client, selected]);

  const currentDetail = selected && detail?.package.ref === selected.package.ref ? detail : null;
  const currentDetailError = currentDetail ? detailError : null;
  const currentDetailLoading = Boolean(
    selected && (detailLoading || !currentDetail),
  );

  return (
    <section aria-labelledby="packages-title">
      <SectionHeading
        id="packages-title"
        title="Package Library"
        description="查看由本地 Catalog 验证过的机器人、世界、控制器、传感器和场景包。"
        action={
          <button className="button button--quiet" type="button" onClick={onReload}>
            <RefreshCw aria-hidden="true" size={16} strokeWidth={1.7} />
            刷新 Catalog
          </button>
        }
      />

      <div className="toolbar" role="search">
        <label className="search-field">
          <span className="sr-only">搜索 Package</span>
          <Search aria-hidden="true" size={17} strokeWidth={1.7} />
          <input
            type="search"
            value={query}
            onChange={(event) => setQuery(event.target.value)}
            placeholder="按名称、类型或描述搜索"
          />
        </label>
        <label className="compact-field">
          <span>类型</span>
          <select value={kind} onChange={(event) => setKind(event.target.value)}>
            <option value="all">全部类型</option>
            {kinds.map((item) => (
              <option value={item} key={item}>
                {item}
              </option>
            ))}
          </select>
        </label>
        <output className="result-count" aria-live="polite">
          {filtered.length} / {packages.length}
        </output>
      </div>

      {error ? <ErrorPanel title="Catalog 加载失败" message={error} onRetry={onReload} /> : null}

      <div className="master-detail">
        <div className="master-pane" aria-label="Package 列表">
          {loading ? <SkeletonRows count={7} /> : null}
          {!loading && !error && filtered.length === 0 ? (
            <EmptyPanel title="没有匹配的 Package">
              修改搜索条件，或先通过受控导入流程添加机器人和世界包。
            </EmptyPanel>
          ) : null}
          {!loading && !error ? (
            <div className="package-list" role="listbox" aria-label="Catalog packages">
              {filtered.map((item) => {
                const key = packageKey(item);
                const selectedItem = key === selectedKey;
                return (
                  <button
                    className={`package-row${selectedItem ? " package-row--selected" : ""}`}
                    type="button"
                    role="option"
                    aria-selected={selectedItem}
                    onClick={() => setSelectedKey(key)}
                    key={key}
                  >
                    <span className="package-row__icon" aria-hidden="true">
                      <Boxes size={18} strokeWidth={1.6} />
                    </span>
                    <span className="package-row__body">
                      <strong>{item.package.id}</strong>
                      <small>{item.description ?? "未提供描述"}</small>
                    </span>
                    <span className="package-row__meta">
                      <span>{item.package.kind}</span>
                      <code>{item.package.version}</code>
                    </span>
                  </button>
                );
              })}
            </div>
          ) : null}
        </div>

        <article className="detail-pane" aria-live="polite">
          {!selected && !loading ? (
            <EmptyPanel title="选择一个 Package">右侧将显示真实 Manifest、依赖和资格信息。</EmptyPanel>
          ) : null}
          {selected ? (
            <>
              <header className="detail-header">
                <div>
                  <span className="kind-label">{selected.package.kind}</span>
                  <h2>{selected.package.ref}</h2>
                  <p>{selected.description ?? "此 Package 没有描述。"}</p>
                </div>
                {currentDetail ? <StatusBadge value={currentDetail.qualification.state} /> : null}
              </header>

              <dl className="fact-grid">
                <div>
                  <dt>Manifest</dt>
                  <dd>{selected.manifest.path}</dd>
                </div>
              </dl>

              {currentDetailLoading ? <SkeletonRows count={3} /> : null}
              {currentDetailError ? <ErrorPanel title="详细信息加载失败" message={currentDetailError} /> : null}

              {currentDetail ? (
                <div className="detail-sections">
                  <section>
                    <h3>
                      <ShieldCheck aria-hidden="true" size={17} strokeWidth={1.7} />
                      Qualification
                    </h3>
                    {currentDetail.qualification.checks.length > 0 ? (
                      <ul className="check-list">
                        {currentDetail.qualification.checks.map((check) => (
                          <li key={check.id}>
                            <span>{check.id}</span>
                            <StatusBadge value={check.status} />
                          </li>
                        ))}
                      </ul>
                    ) : (
                      <p className="muted-copy">尚无资格检查记录，状态为 {currentDetail.qualification.state}。</p>
                    )}
                  </section>

                  <section>
                    <h3>
                      <GitBranch aria-hidden="true" size={17} strokeWidth={1.7} />
                      Dependencies
                    </h3>
                    {currentDetail.dependencies.packages.length > 0 ? (
                      <div className="dependency-list">
                        {currentDetail.dependencies.packages.map((item) => (
                          <code key={`${item.package.kind}:${item.package.ref}`}>
                            {item.package.kind}/{item.package.ref}
                          </code>
                        ))}
                      </div>
                    ) : (
                      <p className="muted-copy">此 Package 没有传递依赖。</p>
                    )}
                  </section>

                  <details className="json-disclosure">
                    <summary>
                      <FileCode2 aria-hidden="true" size={17} strokeWidth={1.7} />
                      查看 Manifest 规范
                    </summary>
                    <pre>{JSON.stringify(currentDetail.manifest_spec, null, 2)}</pre>
                  </details>
                </div>
              ) : null}
            </>
          ) : null}
        </article>
      </div>
    </section>
  );
}
