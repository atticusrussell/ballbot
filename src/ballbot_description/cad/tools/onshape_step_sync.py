#!/usr/bin/env python3
"""
onshape_step_sync.py — move STEP files between Onshape and a code-CAD workflow.

Fills the gap in the community onshape-mcp server (export only, no import):

  elements  List the tabs (elements) in an Onshape document.
  export    Export an Assembly or Part Studio to a local STEP file
            (assembly exports keep parts in assembly coordinates).
  push      Upload a local STEP into Onshape. First push creates a new
            import tab; later pushes of the same file UPDATE that tab in
            place so Onshape tries to keep downstream references/mates.
            Before updating, an Onshape version is created as a restore
            point (disable with --no-snapshot).
  edges     List circular edges (hole rims) of a Part Studio: id, radius,
            center. Use the ids as mate targets.
  mate      Fasten an instance's part origin to a hole-rim edge on another
            instance (PART_ORIGIN -> CENTER connectors). onshape-mcp can't
            build either connector type. Re-running replaces the same mate.

Auth: Onshape API keys via basic auth. Set ONSHAPE_ACCESS_KEY and
ONSHAPE_SECRET_KEY (same variables onshape-mcp uses). Create keys at
https://dev-portal.onshape.com/keys

Document references: paste a normal Onshape browser URL, e.g.
  https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>

State: push records {local file -> blob element id} in
.onshape_imports.json next to the STEP file, so repeat pushes update
instead of creating duplicate tabs. Commit it with your CAD scripts.

Examples:
  python onshape_step_sync.py elements "$DOC_URL"
  python onshape_step_sync.py export "$ASSEMBLY_URL" -o context/chassis.step
  python onshape_step_sync.py push "$DOC_URL" out/motor_bracket.step
  python onshape_step_sync.py push "$DOC_URL" out/motor_bracket.step --new
  python onshape_step_sync.py edges "$PART_STUDIO_URL"
  python onshape_step_sync.py mate "$ASSEMBLY_URL" --origin-of <bracketInst> \
      --to <motorInst>:<edgeId> --name bracket_datum
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import quote, urlparse

import requests

API_VERSION = os.environ.get("ONSHAPE_API_VERSION", "v10")
DEFAULT_BASE = os.environ.get("ONSHAPE_BASE_URL", "https://cad.onshape.com")
STATE_FILE = ".onshape_imports.json"
JSON_ACCEPT = "application/json;charset=UTF-8; qs=0.09"


# --------------------------------------------------------------------------
# URL parsing
# --------------------------------------------------------------------------
@dataclass
class DocRef:
    base: str          # e.g. https://cad.onshape.com
    did: str           # document id
    wvm: str           # "w", "v" or "m"
    wvmid: str         # workspace / version / microversion id
    eid: str | None    # element (tab) id, optional


_URL_RE = re.compile(
    r"/documents/(?P<did>[0-9a-f]{24})/(?P<wvm>[wvm])/(?P<wvmid>[0-9a-f]{24})"
    r"(?:/e/(?P<eid>[0-9a-f]{24}))?"
)


def parse_url(url: str) -> DocRef:
    m = _URL_RE.search(url)
    if not m:
        raise SystemExit(
            f"Couldn't parse Onshape URL: {url}\n"
            "Expected https://cad.onshape.com/documents/<did>/w/<wid>[/e/<eid>]"
        )
    p = urlparse(url)
    base = f"{p.scheme}://{p.netloc}" if p.scheme and p.netloc else DEFAULT_BASE
    return DocRef(base, m["did"], m["wvm"], m["wvmid"], m["eid"])


# --------------------------------------------------------------------------
# API client
# --------------------------------------------------------------------------
class Onshape:
    def __init__(self, base: str, session: requests.Session | None = None):
        ak = os.environ.get("ONSHAPE_ACCESS_KEY")
        sk = os.environ.get("ONSHAPE_SECRET_KEY")
        if not ak or not sk:
            raise SystemExit("Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY.")
        self.api = f"{base}/api/{API_VERSION}"
        self.s = session or requests.Session()
        self.s.auth = (ak, sk)  # HTTP basic auth with API keys

    def _req(self, method: str, path: str, **kw) -> requests.Response:
        headers = kw.pop("headers", {})
        headers.setdefault("Accept", JSON_ACCEPT)
        r = self.s.request(method, self.api + path, headers=headers, timeout=120, **kw)
        if r.status_code >= 400:
            raise SystemExit(f"{method} {path} -> HTTP {r.status_code}\n{r.text[:2000]}")
        return r

    def get(self, path, **kw):
        return self._req("GET", path, **kw)

    def post(self, path, **kw):
        return self._req("POST", path, **kw)

    def delete(self, path, **kw):
        return self._req("DELETE", path, **kw)

    # ---- documents -------------------------------------------------------
    def elements(self, ref: DocRef) -> list[dict]:
        return self.get(f"/documents/d/{ref.did}/{ref.wvm}/{ref.wvmid}/elements").json()

    def create_version(self, ref: DocRef, name: str, description: str = "") -> dict:
        body = {
            "documentId": ref.did,
            "workspaceId": ref.wvmid,
            "name": name,
            "description": description,
        }
        return self.post(f"/documents/d/{ref.did}/versions", json=body).json()

    # ---- translations ----------------------------------------------------
    def wait_translation(self, tid: str, timeout: float = 600) -> dict:
        t0, delay = time.time(), 2.0
        while True:
            t = self.get(f"/translations/{tid}").json()
            state = t.get("requestState")
            if state == "DONE":
                return t
            if state == "FAILED":
                raise SystemExit(f"Translation failed: {t.get('failureReason')}")
            if time.time() - t0 > timeout:
                raise SystemExit(f"Translation {tid} timed out (last state {state}).")
            time.sleep(delay)
            delay = min(delay * 1.6, 20)  # each poll costs an API call

    # ---- export ----------------------------------------------------------
    def export_step(self, ref: DocRef, element_type: str, out: Path,
                    part_ids: list[str] | None = None) -> Path:
        kind = {"ASSEMBLY": "assemblies", "PARTSTUDIO": "partstudios"}.get(element_type)
        if not kind:
            raise SystemExit(f"Can't export element of type {element_type}.")
        body = {
            "formatName": "STEP",
            "storeInDocument": False,
            "flattenAssemblies": False,
        }
        if part_ids:
            body["partIds"] = ",".join(part_ids)
        path = f"/{kind}/d/{ref.did}/{ref.wvm}/{ref.wvmid}/e/{ref.eid}/translations"
        t = self.wait_translation(self.post(path, json=body).json()["id"])
        ext_ids = t.get("resultExternalDataIds") or []
        if not ext_ids:
            raise SystemExit(f"Translation finished but returned no file: {t}")
        data = self.get(
            f"/documents/d/{ref.did}/externaldata/{ext_ids[0]}",
            headers={"Accept": "application/octet-stream"},
        ).content
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(data)
        return out

    # ---- import ----------------------------------------------------------
    def upload_step(self, ref: DocRef, step: Path, update_eid: str | None = None,
                    y_axis_up: bool = False) -> dict:
        if ref.wvm != "w":
            raise SystemExit("Uploads need a workspace URL (/w/), not a version.")
        path = f"/blobelements/d/{ref.did}/w/{ref.wvmid}"
        if update_eid:
            path += f"/e/{update_eid}"
        form = {
            "encodedFilename": quote(step.name),
            "translate": "true",
            "storeInDocument": "true",
            "flattenAssemblies": "false",
            "yAxisIsUp": "true" if y_axis_up else "false",
            "allowFaultyParts": "false",
            "fileContentLength": str(step.stat().st_size),
        }
        with step.open("rb") as fh:
            files = {"file": (step.name, fh, "application/octet-stream")}
            return self.post(path, data=form, files=files).json()


    # ---- part studio queries ---------------------------------------------
    def circle_edges(self, ref: DocRef) -> list[dict]:
        script = (
            "function(context is Context, queries) { var out = []; "
            "for (var e in evaluateQuery(context, qGeometry(qBodyType(qEverything(EntityType.EDGE), BodyType.SOLID), "
            "GeometryType.CIRCLE))) { const c = evCurveDefinition(context, {'edge': e}); "
            "const o = c.coordSystem.origin; const z = c.coordSystem.zAxis; "
            "out = append(out, {'id': transientQueriesToStrings(e), 'r': c.radius / meter, "
            "'x': o[0] / meter, 'y': o[1] / meter, 'z': o[2] / meter, "
            "'nx': z[0], 'ny': z[1], 'nz': z[2]}); } return out; }"
        )
        path = f"/partstudios/d/{ref.did}/{ref.wvm}/{ref.wvmid}/e/{ref.eid}/featurescript"
        res = self.post(path, json={"script": script}).json()["result"]
        return [{kv["key"]["value"]: kv["value"]["value"] for kv in m["value"]}
                for m in res["value"]]

    # ---- assembly mates --------------------------------------------------
    def asm_path(self, ref: DocRef) -> str:
        return f"/assemblies/d/{ref.did}/w/{ref.wvmid}/e/{ref.eid}"

    def add_feature(self, ref: DocRef, feature: dict) -> tuple[str, str]:
        r = self.post(self.asm_path(ref) + "/features", json={"feature": feature}).json()
        return r["feature"]["featureId"], (r.get("featureState") or {}).get("featureStatus")

    def delete_features_named(self, ref: DocRef, names: set[str]) -> int:
        feats = self.get(self.asm_path(ref) + "/features").json()["features"]
        doomed = [f for f in feats if f["name"] in names]
        doomed.sort(key=lambda f: f["btType"] != "BTMMate-64")  # mates before connectors
        for f in doomed:
            self.delete(self.asm_path(ref) + "/features/featureid/" + quote(f["featureId"], safe=""))
        return len(doomed)


def mate_connector(name: str, inst: str, inference: str, det_id: str = "",
                   flip: bool = False) -> dict:
    params = [
        {"btType": "BTMParameterEnum-145", "parameterId": "originType",
         "enumName": "Origin type", "value": "ON_ENTITY"},
        {"btType": "BTMParameterQueryWithOccurrenceList-67", "parameterId": "originQuery",
         "queries": [{"btType": "BTMInferenceQueryWithOccurrence-1083",
                      "inferenceType": inference, "path": [inst],
                      "deterministicIds": [det_id]}]},
    ]
    if flip:
        params.append({"btType": "BTMParameterBoolean-144", "parameterId": "flipPrimary",
                       "value": True})
    return {"btType": "BTMMateConnector-66", "featureType": "mateConnector",
            "name": name, "parameters": params}


def fastened_mate(name: str, mc_a: str, mc_b: str) -> dict:
    return {"btType": "BTMMate-64", "featureType": "mate", "name": name, "parameters": [
        {"btType": "BTMParameterEnum-145", "parameterId": "mateType",
         "enumName": "Mate type", "value": "FASTENED"},
        {"btType": "BTMParameterQueryWithOccurrenceList-67",
         "parameterId": "mateConnectorsQuery",
         "queries": [{"btType": "BTMFeatureQueryWithOccurrence-157", "featureId": f,
                      "path": [], "queryData": ""} for f in (mc_a, mc_b)]},
    ]}


# --------------------------------------------------------------------------
# Local state: which blob element a STEP file was pushed to
# --------------------------------------------------------------------------
def _state_path(step: Path) -> Path:
    return step.resolve().parent / STATE_FILE


def load_state(step: Path) -> dict:
    p = _state_path(step)
    return json.loads(p.read_text()) if p.exists() else {}


def save_state(step: Path, state: dict) -> None:
    _state_path(step).write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")


def state_key(ref: DocRef, step: Path) -> str:
    return f"{ref.did}:{step.name}"


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------
def cmd_elements(args):
    ref = parse_url(args.url)
    for e in Onshape(ref.base).elements(ref):
        print(f"{e.get('elementType', '?'):<12} {e['id']}  {e.get('name', '')}")


def cmd_export(args):
    ref = parse_url(args.url)
    if not ref.eid:
        raise SystemExit("URL must point at a specific tab (/e/<eid>).")
    api = Onshape(ref.base)
    etype = next((e.get("elementType") for e in api.elements(ref) if e["id"] == ref.eid), None)
    out = api.export_step(ref, etype, Path(args.output), args.part_id)
    print(f"Exported {etype} -> {out} ({out.stat().st_size:,} bytes)")


def cmd_push(args):
    ref = parse_url(args.url)
    step = Path(args.step)
    if not step.exists():
        raise SystemExit(f"No such file: {step}")
    if step.suffix.lower() not in (".step", ".stp"):
        raise SystemExit("push expects a .step/.stp file.")
    api = Onshape(ref.base)

    state = load_state(step)
    key = state_key(ref, step)
    target = None if args.new else (args.element or state.get(key, {}).get("blobElementId"))

    existing_ids = {e["id"] for e in api.elements(ref)}
    if target and target not in existing_ids:
        print(f"Recorded import tab {target} no longer exists; creating a new one.")
        target = None

    if target and not args.no_snapshot:
        name = f"pre-{step.stem}-{time.strftime('%Y%m%d-%H%M%S')}"
        api.create_version(ref, name, f"Auto restore point before re-importing {step.name}")
        print(f"Created restore-point version '{name}'.")

    resp = api.upload_step(ref, step, update_eid=target, y_axis_up=args.y_up)
    blob_id = resp.get("id") or target

    # Some responses include a translation id; wait for it if so.
    tid = resp.get("translationId") or (resp.get("translation") or {}).get("id")
    if tid:
        api.wait_translation(tid)

    # Find what the import produced (Part Studio / Assembly tabs) by diffing.
    new_tabs = []
    if not target:
        deadline, delay = time.time() + args.wait, 2.0
        while time.time() < deadline:
            new_tabs = [e for e in api.elements(ref)
                        if e["id"] not in existing_ids and e["id"] != blob_id]
            if any(e.get("elementType") in ("PARTSTUDIO", "ASSEMBLY") for e in new_tabs):
                break
            time.sleep(delay)
            delay = min(delay * 1.6, 20)  # each poll costs an API call

    rec = state.get(key, {})
    rec.update({"blobElementId": blob_id, "documentId": ref.did,
                "lastPush": time.strftime("%Y-%m-%dT%H:%M:%S")})
    if new_tabs:
        rec["resultElements"] = [{"id": e["id"], "type": e.get("elementType"),
                                  "name": e.get("name")} for e in new_tabs]
    state[key] = rec
    save_state(step, state)

    verb = "Updated" if target else "Created"
    print(f"{verb} import tab {blob_id} from {step.name}.")
    for e in rec.get("resultElements", []):
        print(f"  -> {e['type']:<11} {e['id']}  {e['name']}")
    if not target and not new_tabs:
        print("  (translation may still be running; check the document)")
    print(f"State saved to {_state_path(step)}")


def cmd_edges(args):
    ref = parse_url(args.url)
    if not ref.eid:
        raise SystemExit("URL must point at a Part Studio tab (/e/<eid>).")
    for e in Onshape(ref.base).circle_edges(ref):
        print(f"{e['id']:<6} r={e['r'] * 1000:7.3f}  center=({e['x'] * 1000:8.3f}, "
              f"{e['y'] * 1000:8.3f}, {e['z'] * 1000:8.3f}) mm  "
              f"axis=({e['nx']:+.0f}, {e['ny']:+.0f}, {e['nz']:+.0f})")


def cmd_mate(args):
    ref = parse_url(args.url)
    if not ref.eid or ref.wvm != "w":
        raise SystemExit("URL must point at an Assembly tab in a workspace (/w/.../e/<eid>).")
    target_inst, _, edge = args.to.rpartition(":")
    if not target_inst or not edge:
        raise SystemExit("--to must be <instanceId>:<edgeId>")
    api = Onshape(ref.base)
    n_origin, n_target = f"{args.name} origin", f"{args.name} target"
    removed = api.delete_features_named(ref, {args.name, n_origin, n_target})
    if removed:
        print(f"Replaced {removed} existing feature(s) named '{args.name}*'.")
    a = api.add_feature(ref, mate_connector(n_origin, args.origin_of, "PART_ORIGIN"))
    b = api.add_feature(ref, mate_connector(n_target, target_inst, "CENTER", edge, args.flip))
    m = api.add_feature(ref, fastened_mate(args.name, a[0], b[0]))
    for label, (fid, st) in ((n_origin, a), (n_target, b), (args.name, m)):
        print(f"  {label:<28} {fid}  {st}")
    if any(st != "OK" for _, st in (a, b, m)):
        raise SystemExit("Mate did not resolve; check instance/edge ids.")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("elements", help="list tabs in a document")
    p.add_argument("url")
    p.set_defaults(func=cmd_elements)

    p = sub.add_parser("export", help="export an assembly/part studio tab to STEP")
    p.add_argument("url", help="Onshape URL of the Assembly or Part Studio tab")
    p.add_argument("-o", "--output", required=True, help="output .step path")
    p.add_argument("--part-id", action="append",
                   help="Part Studio only: export just this part id (repeatable)")
    p.set_defaults(func=cmd_export)

    p = sub.add_parser("push", help="upload/update a STEP as an Onshape import")
    p.add_argument("url", help="Onshape URL of the target document workspace")
    p.add_argument("step", help="local .step file")
    p.add_argument("--new", action="store_true", help="always create a new import tab")
    p.add_argument("--element", help="update this specific blob element id")
    p.add_argument("--no-snapshot", action="store_true",
                   help="skip creating a restore-point version before updating")
    p.add_argument("--y-up", action="store_true", help="treat the file's Y axis as up")
    p.add_argument("--wait", type=float, default=120,
                   help="seconds to wait for the import tab to appear (default 120)")
    p.set_defaults(func=cmd_push)

    p = sub.add_parser("edges", help="list circular edges (hole rims) in a Part Studio")
    p.add_argument("url", help="Onshape URL of the Part Studio tab")
    p.set_defaults(func=cmd_edges)

    p = sub.add_parser("mate", help="fasten a part origin to a hole-rim edge")
    p.add_argument("url", help="Onshape URL of the Assembly tab")
    p.add_argument("--origin-of", required=True, help="instance id whose part origin is the datum")
    p.add_argument("--to", required=True, help="<instanceId>:<edgeId> of the target hole rim")
    p.add_argument("--name", default="code_part_datum", help="mate name (re-run replaces it)")
    p.add_argument("--flip", action="store_true", help="flip the target connector's Z")
    p.set_defaults(func=cmd_mate)

    args = ap.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
