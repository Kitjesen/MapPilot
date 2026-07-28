"""Single operational control plane for compiled Products."""

from __future__ import annotations

import argparse
import json
import os
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from lingtu.assembly.profile_builder import compile_product
from lingtu.launcher import Launcher, LaunchFailed, LaunchReport
from lingtu.product import Product, ProductManifest
from lingtu.product_switch import (
    SwitchBackend,
    SwitchFailed,
    SwitchReport,
    SwitchRequest,
    execute_switch,
)
from runtime.profiles.resolver import resolve_runtime_config


class ProductControl:
    """Operate the current product without reconstructing process targets."""

    def __init__(
        self,
        launcher: Launcher | None = None,
        *,
        environment: Mapping[str, str] | None = None,
    ) -> None:
        self._launcher = launcher
        self._environment = environment if environment is not None else os.environ
        self._products: dict[tuple[str, str | None], Product] = {}

    def product(
        self,
        profile: str | None = None,
        *,
        endpoint: str | None = None,
    ) -> Product:
        """Compile one Product per profile/endpoint pair and reuse it."""

        profile = str(profile or self._environment.get("LINGTU_PROFILE") or "").strip()
        if not profile:
            raise RuntimeError("LINGTU_PROFILE is required for product process control")
        endpoint = str(endpoint or self._environment.get("LINGTU_ENDPOINT") or "").strip() or None
        requested = (profile, endpoint)
        cached = self._products.get(requested)
        if cached is not None:
            return cached
        resolved = resolve_runtime_config(profile, runtime_endpoint_name=endpoint)
        product = compile_product(
            resolved.profile,
            resolved.config,
            endpoint=resolved.runtime_endpoint,
        )
        canonical_key = (product.profile, product.endpoint)
        self._products[requested] = product
        self._products[canonical_key] = product
        return product

    def write_manifest(
        self,
        path: str | Path,
        *,
        profile: str | None = None,
        endpoint: str | None = None,
    ) -> ProductManifest:
        """Compile once and atomically publish the resulting contract."""

        manifest = self.product(profile, endpoint=endpoint).manifest()
        manifest.write(path)
        return manifest

    def switch(
        self,
        request: SwitchRequest,
        *,
        backend: SwitchBackend | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> SwitchReport:
        """Apply one complete fail-closed Product transition."""

        return execute_switch(
            self,
            request,
            backend=backend,
            environment=self._environment,
            state_dir=state_dir,
            dry_run=dry_run,
        )

    def apply(
        self,
        *,
        profile: str | None = None,
        endpoint: str | None = None,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Compile and apply one Product."""

        return self.apply_product(
            self.product(profile, endpoint=endpoint),
            dry_run=dry_run,
        )

    def apply_product(
        self,
        product: Product | ProductManifest,
        *,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Apply an already compiled Product contract."""

        if product.process_control == "launcher":
            raise RuntimeError(
                "field Product requires ProductControl.switch; "
                "direct apply would bypass manifest publication and runtime staging"
            )
        return (self._launcher or Launcher()).apply(product, dry_run=dry_run)

    def apply_manifest(
        self,
        path: str | Path,
        *,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Apply exactly the fingerprinted Product that passed preflight."""

        manifest = ProductManifest.load(path)
        return (self._launcher or Launcher()).apply(manifest, dry_run=dry_run)

    def stop(
        self,
        *,
        profile: str | None = None,
        endpoint: str | None = None,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Compile and stop one Product's mode-owned processes."""

        return self.stop_product(
            self.product(profile, endpoint=endpoint),
            dry_run=dry_run,
        )

    def stop_product(
        self,
        product: Product | ProductManifest,
        *,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Stop processes declared by an already compiled Product."""

        return (self._launcher or Launcher()).stop(product, dry_run=dry_run)

    def stop_manifest(
        self,
        path: str | Path,
        *,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Stop processes declared by a persisted Product manifest."""

        return self.stop_product(ProductManifest.load(path), dry_run=dry_run)

    def quiesce_product(
        self,
        product: Product | ProductManifest,
        *,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Stop every mode process that may conflict with a failed switch."""

        return (self._launcher or Launcher()).quiesce(product, dry_run=dry_run)

    def restart(
        self,
        process_name: str,
        *,
        profile: str | None = None,
        endpoint: str | None = None,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Compile a Product and restart one declared process."""

        return self.restart_product(
            self.product(profile, endpoint=endpoint),
            process_name,
            dry_run=dry_run,
        )

    def restart_product(
        self,
        product: Product | ProductManifest,
        process_name: str,
        *,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Restart one process in an already compiled Product."""

        return (self._launcher or Launcher()).restart(
            product,
            process_name,
            dry_run=dry_run,
        )

    def restart_manifest(
        self,
        path: str | Path,
        process_name: str,
        *,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Restart one process declared by a persisted manifest."""

        return self.restart_product(
            ProductManifest.load(path),
            process_name,
            dry_run=dry_run,
        )

def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("plan", "apply", "stop", "restart", "switch"))
    parser.add_argument("profile", nargs="?")
    parser.add_argument("--endpoint", "--target-endpoint", dest="endpoint")
    parser.add_argument("--current")
    parser.add_argument("--map", dest="map_name")
    parser.add_argument("--process")
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--manifest-out", type=Path)
    parser.add_argument("--state-dir", type=Path)
    parser.add_argument("--initial-pose", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    relocalize = parser.add_mutually_exclusive_group()
    relocalize.add_argument("--relocalize", dest="relocalize", action="store_true")
    relocalize.add_argument("--no-relocalize", dest="relocalize", action="store_false")
    parser.set_defaults(relocalize=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser


def _manifest_or_product(
    control: ProductControl,
    args: argparse.Namespace,
) -> Product | ProductManifest:
    if args.manifest is not None:
        if args.profile or args.endpoint:
            raise ValueError("--manifest cannot be combined with profile or --endpoint")
        return ProductManifest.load(args.manifest)
    product = control.product(args.profile, endpoint=args.endpoint)
    if args.manifest_out is not None:
        product.manifest().write(args.manifest_out)
    return product


def _print(payload: Mapping[str, Any], *, json_output: bool) -> None:
    if json_output:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return
    status = str(payload.get("status") or "planned")
    product = str(payload.get("product") or payload.get("profile") or payload.get("target_profile") or "unknown")
    endpoint = str(payload.get("endpoint") or "in_process")
    print(f"{status}: {product} on {endpoint}")


def main(argv: list[str] | None = None) -> int:
    """Run Product control outside the managed Host process."""

    args = _parser().parse_args(argv)
    try:
        control = ProductControl()
        if args.action == "switch":
            if args.manifest is not None or args.manifest_out is not None:
                raise ValueError("switch compiles and publishes its own Product manifest")
            initial_pose = tuple(args.initial_pose) if args.initial_pose is not None else None
            payload = control.switch(
                SwitchRequest(
                    target_profile=str(args.profile or ""),
                    endpoint=str(args.endpoint or "thunder_field"),
                    current_profile=args.current,
                    map_name=args.map_name,
                    relocalize=bool(args.relocalize),
                    initial_pose=initial_pose,
                ),
                state_dir=args.state_dir,
                dry_run=args.dry_run,
            ).as_dict()
        else:
            product = _manifest_or_product(control, args)
        if args.action == "plan":
            payload = product.as_dict()
            payload["status"] = "planned"
        elif args.action == "apply":
            payload = control.apply_product(product, dry_run=args.dry_run).as_dict()
        elif args.action == "stop":
            payload = control.stop_product(product, dry_run=args.dry_run).as_dict()
        elif args.action == "restart":
            if not args.process:
                raise ValueError("restart requires --process <logical-name>")
            payload = control.restart_product(
                product,
                args.process,
                dry_run=args.dry_run,
            ).as_dict()
        elif args.action == "switch":
            pass
        else:
            raise ValueError(f"unsupported Product action: {args.action}")
    except SwitchFailed as exc:
        print(json.dumps(exc.report.as_dict(), ensure_ascii=False, indent=2))
        return 1
    except LaunchFailed as exc:
        print(json.dumps(exc.report.as_dict(), ensure_ascii=False, indent=2))
        return 1
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False, indent=2))
        return 2
    _print(payload, json_output=args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
