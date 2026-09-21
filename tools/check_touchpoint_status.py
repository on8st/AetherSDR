#!/usr/bin/env python3
"""
AetherSDR aetherd-migration conversion ratchet — "converted only grows".

Guards the one property the burndown depends on
(docs/aetherd-agents-md-staging.md, "Ratchets"):

    The touchpoint manifest's "converted" column only ever grows.

A migration converts touchpoints. It never un-converts one. A row that
read `converted:#4099` on the merge base and no longer does on the head
is a DROPPED CONVERSION — the defect this check exists to catch — and it
is invisible in review, because the diff that causes it is one word in a
229-entry JSON file and the manifest's totals line moves by one digit.

The ledger being compared is the status sidecar:

    docs/architecture/aetherd-touchpoint-status.json

one row per touchpoint key (`<module>/<header>`, exactly the keys
docs/architecture/aetherd-touchpoint-tags.json uses), whose value is one
of the three forms tools/gen_touchpoint_manifest.py documents:

    "unconverted" | "in progress:<who>" | "converted:<PR#>"

WHAT IS COMPARED, AND WHY IT IS THE SET AND NOT THE COUNT. A count floor
cannot see a lateral swap — drop `converted` on `core/RadioModel.h`, add
it on `core/MeterModel.h`, and the total is flat while a conversion was
lost. So the check is per-key: every key converted on the base must
still be converted on the head. This is the same set-not-count reasoning
EB3's vendor baseline is built on (tools/check_engine_boundary.py).

THE LEDGER IS APPEND-ONLY, AND A ROW IS NEVER DELETED. A header can stop
being a live touchpoint — that is what a finished conversion looks like
from the manifest's side, since the UI no longer includes it and the
scan no longer lists it. Its status row stays behind as history. The
generator only ever looks up keys it scanned, so a stale row costs
nothing, and keeping it is what lets this check treat "the row is gone"
as unambiguously a regression rather than maybe-a-cleanup. Rows are
therefore NOT validated against the live touchpoint set here; a key the
scan no longer reports is reported as a note, never as a failure.

MALFORMED VALUES FAIL, AND THAT IS NOT PEDANTRY. The manifest counts a
conversion with `str(status.get(key, "")).startswith("converted")` —
case-sensitive, prefix-only. `Converted:#4099` and `done:#4099` are
therefore silently NOT conversions: the row looks converted to a human
reading the JSON and does not count anywhere else. Any value outside the
three documented forms is an error here so that gap cannot open.

NO --strict FLAG, ON PURPOSE. Every finding this script can produce is a
hard regression; none of them is advisory. The sibling checkers carry
--strict because they also emit warnings against tracked baselines that
must NOT block. Giving this one an advisory default would reproduce the
failure static-checks.yml documents at its head — a gate that existed
and went unenforced for eight days because nothing checks whether the
enforcing flag is actually passed. It exits non-zero on its own.

Exit 0 clean; 1 on a violation; 2 if the base revision cannot be
resolved (the ratchet could not run, which is not the same as passing
and must not be reported as one).

Usage:
    python tools/check_touchpoint_status.py
    python tools/check_touchpoint_status.py --base origin/main

stdlib only; no third-party dependencies.
"""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
STATUS_JSON = REPO / "docs" / "architecture" / "aetherd-touchpoint-status.json"
STATUS_REL = STATUS_JSON.relative_to(REPO).as_posix()

# The three documented forms, and nothing else. The `converted` arm allows a
# trailing remark after the PR number (`converted:#4099 (partial revert
# pending)`) because the manifest renders the value verbatim; it does NOT allow
# the prefix to vary, because the manifest's count keys on exactly that prefix.
# Bare `converted` with no PR number is rejected deliberately: the manifest's
# prefix test would count it, and a conversion that cites no PR cannot be
# audited back to the change that made it.
#
# `|` is excluded from every arm because the value is interpolated verbatim
# into a `|`-delimited markdown row by the generator's render(). A pipe there
# silently splits one row into two columns — the table still renders, so
# nothing downstream notices, and the burndown quietly misreads.
STATUS_RE = re.compile(
    r"^(?:unconverted"
    r"|in progress:[^|\s][^|]*"
    r"|converted:#?\d+(?:\s[^|]*)?)$"
)

CONVERTED_PREFIX = "converted"


def is_converted(value):
    """Mirror the manifest's own test, so the two can never disagree.

    tools/gen_touchpoint_manifest.py counts a conversion with
    str(value).startswith("converted"). Anything that changes here must
    change there; the test pins that they agree.
    """
    return str(value).startswith(CONVERTED_PREFIX)


def converted_keys(status):
    return {k for k, v in status.items() if is_converted(v)}


def parse_status(text, where):
    """Strict parse. A malformed ledger is an error, never an empty dict.

    The generator deliberately degrades to {} on bad JSON so a broken
    sidecar cannot wedge regeneration. Inheriting that here would make
    this check pass vacuously on exactly the commit that corrupted the
    ledger, which is the one commit it must not pass.
    """
    if text is None:
        return None
    try:
        data = json.loads(text)
    except (json.JSONDecodeError, ValueError) as e:
        raise ValueError(f"{where} is not valid JSON: {e}") from None
    if not isinstance(data, dict):
        raise ValueError(f"{where} must be a JSON object, got {type(data).__name__}")
    return data


def git(args, cwd=REPO):
    return subprocess.run(
        ["git"] + args, cwd=str(cwd),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )


def resolve_base(explicit):
    """Return (rev, description) or (None, reason)."""
    if explicit:
        candidates = [explicit]
    elif os.environ.get("GITHUB_BASE_REF"):
        base = os.environ["GITHUB_BASE_REF"]
        candidates = [f"origin/{base}", base]
    else:
        candidates = ["origin/main", "main"]

    for ref in candidates:
        if git(["rev-parse", "--verify", "--quiet", f"{ref}^{{commit}}"]).returncode != 0:
            continue
        mb = git(["merge-base", ref, "HEAD"])
        if mb.returncode == 0 and mb.stdout.strip():
            return mb.stdout.strip(), f"merge-base with {ref}"
        # Detached or unrelated history: compare against the ref itself
        # rather than giving up, but say so.
        return ref, f"{ref} (no merge-base with HEAD)"
    return None, f"none of {', '.join(candidates)} resolves in this clone"


def status_at(rev):
    """Ledger content at `rev`, or None if the file does not exist there."""
    r = git(["show", f"{rev}:{STATUS_REL}"])
    if r.returncode != 0:
        return None
    return r.stdout


def main():
    ap = argparse.ArgumentParser(
        description="Fail if any converted aetherd touchpoint stopped being "
                    "converted relative to the merge base.")
    ap.add_argument("--base", metavar="REV",
                    help="base revision to compare against "
                         "(default: merge-base with origin/<PR base>, "
                         "else origin/main)")
    args = ap.parse_args()

    # ---- head side -------------------------------------------------------
    if not STATUS_JSON.is_file():
        print(f"::error::{STATUS_REL} is missing — the conversion ledger the "
              "aetherd burndown is counted from cannot be read")
        return 1
    try:
        head = parse_status(STATUS_JSON.read_text(), STATUS_REL)
    except ValueError as e:
        print(f"::error::{e}")
        return 1

    bad = sorted(k for k, v in head.items()
                 if not isinstance(v, str) or not STATUS_RE.fullmatch(v))
    if bad:
        for k in bad:
            print(f"::error::{STATUS_REL}: {k} has status "
                  f"{head[k]!r}, which is not one of 'unconverted', "
                  "'in progress:<who>', 'converted:<PR#>' — the manifest "
                  "counts conversions by exact prefix, so this row counts as "
                  "unconverted no matter how it reads")
        return 1

    # ---- base side -------------------------------------------------------
    base_rev, how = resolve_base(args.base)
    if base_rev is None:
        print(f"::error::cannot resolve a base revision to compare against "
              f"({how}); the converted-only-grows ratchet did NOT run")
        return 2

    base_text = status_at(base_rev)
    if base_text is None:
        print(f"{STATUS_REL} does not exist at {base_rev[:12]} ({how}); "
              "treating the base ledger as empty — nothing can have been "
              "dropped")
        base = {}
    else:
        try:
            base = parse_status(base_text, f"{STATUS_REL} at {base_rev[:12]}")
        except ValueError as e:
            print(f"::error::{e}")
            return 1

    # ---- the ratchet -----------------------------------------------------
    was = converted_keys(base)
    now = converted_keys(head)
    dropped = sorted(was - now)

    if dropped:
        for k in dropped:
            before = base[k]
            after = head.get(k, "<row deleted>")
            print(f"::error::{STATUS_REL}: {k} was {before!r} on the base and "
                  f"is {after!r} on this branch — a converted touchpoint may "
                  "never become unconverted. If this conversion was genuinely "
                  "reverted, that is a maintainer decision, not a PR edit.")
        print(f"::error::converted-only-grows: {len(dropped)} conversion(s) "
              f"dropped against {how} ({len(was)} converted on the base, "
              f"{len(now)} here)")
        return 1

    gained = sorted(now - was)
    if gained:
        print(f"converted-only-grows: +{len(gained)} "
              f"({', '.join(gained)}) — {len(was)} → {len(now)} converted "
              f"against {how}")
    else:
        print(f"converted-only-grows: {len(now)} converted, unchanged "
              f"against {how}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
