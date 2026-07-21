"""stack_manifest — freeze and VERIFY the actual running binaries + environment (07-21 ruling #3).

A git tag freezes sources, but the runtime loads git-IGNORED hand-built binaries
(ego/capi/ego_capi.so, cpp/capi/sando_capi.so), so a tag alone cannot prove WHICH binary an
experiment loaded. This tool:
  --emit --label X   records sha256 of every runtime .so, the source git SHA (full), and the
                     external MetaUrban repo SHA into out/baselines/stack_manifest_<label>.json
  --verify PATH      recomputes everything and FAIL-CLOSES loudly on any mismatch
runtime_shas() is the shared helper harvest/final100 receipts embed, and ego_bridge verifies the
loaded .so at import when STACK_MANIFEST=<path> is set.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys

SC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SO_FILES = ("ego/capi/ego_capi.so", "cpp/capi/sando_capi.so", "cpp/capi/cert_capi.so")
METAURBAN = "/media/boxuan/Data2/projects/metaurban"


def _sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _git(cwd):
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=cwd, text=True).strip()
    except Exception:
        return "unknown"


def runtime_shas():
    """The shas every receipt embeds: source git (full), each runtime .so, MetaUrban env."""
    out = dict(git_sha=_git(SC), metaurban_sha=_git(METAURBAN), so_sha256={})
    for rel in SO_FILES:
        p = os.path.join(SC, rel)
        out["so_sha256"][os.path.basename(rel)] = _sha(p) if os.path.exists(p) else "MISSING"
    return out


def verify(path):
    """Recompute and compare; raise loudly on ANY mismatch (fail-closed)."""
    want = json.load(open(path))
    have = runtime_shas()
    bad = []
    for k in ("git_sha", "metaurban_sha"):
        if want.get(k) != have[k]:
            bad.append(f"{k}: {have[k][:12]} != frozen {str(want.get(k))[:12]}")
    for name, sha in (want.get("so_sha256") or {}).items():
        if have["so_sha256"].get(name) != sha:
            bad.append(f"{name}: {str(have['so_sha256'].get(name))[:12]} != frozen {sha[:12]}")
    if bad:
        raise AssertionError("STACK_MANIFEST violation (the running stack is NOT the frozen one):\n  "
                             + "\n  ".join(bad))
    return True


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--emit", action="store_true")
    ap.add_argument("--label", default=None)
    ap.add_argument("--verify", default=None)
    a = ap.parse_args()
    if a.emit:
        assert a.label, "--emit needs --label"
        rec = runtime_shas()
        rec["label"] = a.label
        out = os.path.join(SC, "out", "baselines", f"stack_manifest_{a.label}.json")
        tmp = out + ".tmp"
        json.dump(rec, open(tmp, "w"), indent=1)
        os.replace(tmp, out)
        print(f"[stack] manifest -> {out}")
        for k, v in rec["so_sha256"].items():
            print(f"[stack]   {k}: {v[:16]}")
        print(f"[stack]   git={rec['git_sha'][:12]} metaurban={rec['metaurban_sha'][:12]}")
    elif a.verify:
        verify(a.verify)
        print(f"[stack] VERIFIED against {a.verify}")
    else:
        ap.print_help(); sys.exit(2)
