#!/usr/bin/env python3
import argparse
import os
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


class SidecarRequestHandler(SimpleHTTPRequestHandler):
    """Serve sidecar web assets and package-share files via /pkg/<package>/<path>."""

    def __init__(self, *args, web_root: str, **kwargs):
        self._web_root = Path(web_root).resolve()
        super().__init__(*args, directory=str(self._web_root), **kwargs)

    def _infer_install_root(self):
        # Typical path: <ws>/install/<pkg>/share/<pkg>/web/taxel_sidecar
        for parent in [self._web_root, *self._web_root.parents]:
            if parent.name == "install":
                return parent
        return None

    def _candidate_prefixes(self):
        prefixes = []
        install_root = self._infer_install_root()
        if install_root is not None:
            prefixes.append(install_root)

        for env_name in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
            for value in os.environ.get(env_name, "").split(os.pathsep):
                if value:
                    prefixes.append(Path(value))

        # De-duplicate while preserving order.
        uniq = []
        seen = set()
        for prefix in prefixes:
            key = str(prefix)
            if key in seen:
                continue
            seen.add(key)
            uniq.append(prefix)
        return uniq

    def _resolve_package_share_dir(self, package_name: str):
        try:
            return Path(get_package_share_directory(package_name)).resolve()
        except PackageNotFoundError:
            pass

        for prefix in self._candidate_prefixes():
            # Isolated install layout: <install>/<pkg>/share/<pkg>
            isolated = prefix / package_name / "share" / package_name
            if isolated.is_dir():
                return isolated.resolve()
            # Merged install layout: <install>/share/<pkg>
            merged = prefix / "share" / package_name
            if merged.is_dir():
                return merged.resolve()
        return None

    def do_GET(self):
        parsed = urlparse(self.path)
        path = unquote(parsed.path)

        if path.startswith("/pkg/"):
            self._serve_package_asset(path)
            return

        super().do_GET()

    def end_headers(self):
        # Disable browser caching so web UI updates are immediately visible during development.
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def _serve_package_asset(self, path: str):
        # Expected: /pkg/<package>/<relative/path>
        parts = path.split("/")
        if len(parts) < 4 or not parts[2]:
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid /pkg path")
            return

        package_name = parts[2]
        raw_relative_path = "/".join(parts[3:])
        # Normalize URL path fragments so values like "//mesh/foo.stl" are treated as "mesh/foo.stl".
        normalized_parts = [p for p in raw_relative_path.replace("\\", "/").split("/") if p not in ("", ".")]
        if any(p == ".." for p in normalized_parts):
            self.send_error(HTTPStatus.FORBIDDEN, "Path traversal is not allowed")
            return
        if not normalized_parts:
            self.send_error(HTTPStatus.BAD_REQUEST, "Missing package file path")
            return
        relative_path = "/".join(normalized_parts)

        share_dir = self._resolve_package_share_dir(package_name)
        if share_dir is None:
            self.send_error(HTTPStatus.NOT_FOUND, f"Package not found: {package_name}")
            return

        target = Path(os.path.normpath(str((share_dir / relative_path).absolute())))
        # Keep the non-resolving check to allow symlink-install package assets.
        if not str(target).startswith(str(share_dir)):
            self.send_error(HTTPStatus.FORBIDDEN, "Path traversal is not allowed")
            return

        if not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND, f"File not found: {relative_path}")
            return

        try:
            with target.open("rb") as f:
                fs = os.fstat(f.fileno())
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-type", self.guess_type(str(target)))
                self.send_header("Content-Length", str(fs.st_size))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.copyfile(f, self.wfile)
        except OSError as exc:
            self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, f"I/O error: {exc}")


def parse_args():
    parser = argparse.ArgumentParser(description="Xela taxel sidecar HTTP server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--web-root", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    handler = partial(SidecarRequestHandler, web_root=args.web_root)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"Serving sidecar web root: {Path(args.web_root).resolve()}")
    print(f"Serving package assets at: /pkg/<package>/<path>")
    print(f"HTTP server listening on http://{args.host}:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
