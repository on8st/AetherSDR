"""Regression cases for the aetherd converted-only-grows ratchet.

TWO ARMS, DELIBERATELY. The unit arm exercises the comparison directly.
The end-to-end arm builds a throwaway git repository, commits a ledger,
changes it on a branch, and runs tools/check_touchpoint_status.py as a
SUBPROCESS, asserting on its exit status.

The second arm exists because the first cannot fail for the reason that
matters most. A ratchet that computes the right answer and then exits 0
anyway is the defect that actually ships — a harness reporting green on
a red run — and no amount of testing the comparison function catches it.
So the end-to-end cases assert the process exit code, and one of them
asserts that the base-revision plumbing genuinely reads the OLD file
rather than the working tree twice.

Socket-free, git-only, stdlib-only; no radio and no network.
"""
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CHECKER_PATH = REPO / "tools" / "check_touchpoint_status.py"
GEN_PATH = REPO / "tools" / "gen_touchpoint_manifest.py"
STATUS_REL = "docs/architecture/aetherd-touchpoint-status.json"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


checker = load(CHECKER_PATH, "check_touchpoint_status")


class ConvertedSetTest(unittest.TestCase):
    """The comparison itself."""

    def test_converted_is_prefix_and_case_sensitive(self):
        self.assertTrue(checker.is_converted("converted:#4099"))
        self.assertTrue(checker.is_converted("converted:4099"))
        self.assertFalse(checker.is_converted("unconverted"))
        self.assertFalse(checker.is_converted("in progress:on8st"))
        # The trap this guards: reads as converted, counts as nothing.
        self.assertFalse(checker.is_converted("Converted:#4099"))

    def test_agrees_with_the_manifest_generator(self):
        """Pin that the two prefix tests cannot drift apart.

        The manifest counts conversions itself; if this checker's notion
        of "converted" ever diverged from the generator's, the ratchet
        would guard a number nobody reads. Rather than restate the
        generator's rule (a test that copies the constant agrees with
        itself), render a real one-row manifest through the generator
        and read the count back out of its totals line.
        """
        gen = load(GEN_PATH, "gen_touchpoint_manifest")
        tps = {("core", "Probe.h"): ["src/gui/X.cpp"]}
        tags = {"core/Probe.h": {"tag": "universal"}}
        for value, expected in [("converted:#1", "1/1 converted"),
                                ("Converted:#1", "0/1 converted"),
                                ("unconverted", "0/1 converted"),
                                ("in progress:x", "0/1 converted")]:
            out = gen.render(tps, tags, {"core/Probe.h": value})
            self.assertIn(expected, out, f"generator disagreed for {value!r}")
            self.assertEqual(
                checker.is_converted(value), expected.startswith("1/"),
                f"checker disagreed with the generator for {value!r}")

    def test_lateral_swap_is_a_drop(self):
        """A flat count hides this; the set does not."""
        base = {"core/A.h": "converted:#1", "core/B.h": "unconverted"}
        head = {"core/A.h": "unconverted", "core/B.h": "converted:#2"}
        self.assertEqual(len(checker.converted_keys(base)),
                         len(checker.converted_keys(head)))
        self.assertEqual(
            sorted(checker.converted_keys(base) - checker.converted_keys(head)),
            ["core/A.h"])

    def test_malformed_values_are_rejected(self):
        for good in ["unconverted", "in progress:on8st", "converted:#4099",
                     "converted:4099", "converted:#4099 (audio half only)"]:
            self.assertIsNotNone(checker.STATUS_RE.fullmatch(good), good)
        for bad in ["Converted:#4099", "done:#4099", "converted", "",
                    "in progress:", "converted:#", "CONVERTED:#1", " unconverted",
                    # A pipe splits the generated markdown row in two.
                    "in progress:on8st | maybe", "converted:#1 (a|b)"]:
            self.assertIsNone(checker.STATUS_RE.fullmatch(bad), bad)

    def test_a_pipe_would_really_break_the_generated_row(self):
        """Not a style rule — prove the damage the regex prevents.

        A control that only asserted the regex rejects `|` would agree
        with itself. Render the row through the real generator and count
        the table columns.
        """
        gen = load(GEN_PATH, "gen_touchpoint_manifest")
        tps = {("core", "Probe.h"): ["src/gui/X.cpp"]}
        tags = {"core/Probe.h": {"tag": "universal"}}

        def row(value):
            out = gen.render(tps, tags, {"core/Probe.h": value})
            return next(ln for ln in out.splitlines()
                        if ln.startswith("| `core/Probe.h`"))

        self.assertEqual(row("converted:#1").count("|"),
                         row("unconverted").count("|"))
        self.assertGreater(row("converted:#1 (a|b)").count("|"),
                           row("converted:#1").count("|"))

    def test_bad_json_is_an_error_not_an_empty_ledger(self):
        with self.assertRaises(ValueError):
            checker.parse_status("{ not json", "x")
        with self.assertRaises(ValueError):
            checker.parse_status("[]", "x")


class EndToEndTest(unittest.TestCase):
    """Run the real script against a real repository and read exit codes."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull,
                   GIT_CONFIG_SYSTEM=os.devnull)
        self.env = {k: v for k, v in env.items() if k != "GITHUB_BASE_REF"}
        self.git("init", "-q", "-b", "main")
        self.git("config", "user.email", "t@example.invalid")
        self.git("config", "user.name", "t")
        # The checker locates the repo from its own path, so it must live
        # inside the fixture tree at the same relative position.
        (self.root / "tools").mkdir()
        (self.root / "docs" / "architecture").mkdir(parents=True)
        (self.root / "tools" / "check_touchpoint_status.py").write_bytes(
            CHECKER_PATH.read_bytes())
        self.script = self.root / "tools" / "check_touchpoint_status.py"

    def tearDown(self):
        self.tmp.cleanup()

    def git(self, *args):
        r = subprocess.run(["git"] + list(args), cwd=self.root, env=self.env,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True)
        self.assertEqual(r.returncode, 0, r.stdout)
        return r.stdout

    def write_status(self, mapping):
        (self.root / STATUS_REL).write_text(json.dumps(mapping, indent=1) + "\n")

    def commit(self, message):
        self.git("add", "--", STATUS_REL)
        self.git("commit", "-q", "-m", message)
        return self.git("rev-parse", "HEAD").strip()

    def run_checker(self, base=None):
        cmd = [sys.executable, str(self.script)]
        if base:
            cmd += ["--base", base]
        return subprocess.run(cmd, cwd=self.root, env=self.env,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True)

    def test_dropping_a_conversion_exits_1_and_names_it(self):
        self.write_status({"core/A.h": "converted:#4099",
                           "core/B.h": "unconverted"})
        base = self.commit("base: A converted")

        self.write_status({"core/A.h": "unconverted",
                           "core/B.h": "unconverted"})
        self.commit("head: A un-converted")

        r = self.run_checker(base=base)
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn("core/A.h", r.stdout)
        self.assertIn("converted:#4099", r.stdout)

    def test_lateral_swap_exits_1_though_the_count_is_flat(self):
        """The case a count-based ratchet passes and must not.

        This test exists because an earlier draft of this file did NOT
        have it: perturbing the checker to compare len(converted) instead
        of the key sets left all fifteen other cases green. The unit case
        above exercises converted_keys() directly, which is not the code
        path `main` takes, so it could not see the regression. Drive the
        real process, with the total deliberately unchanged.
        """
        self.write_status({"core/A.h": "converted:#4099",
                           "core/B.h": "unconverted"})
        base = self.commit("base: A converted")

        self.write_status({"core/A.h": "unconverted",
                           "core/B.h": "converted:#4100"})
        self.commit("head: swapped which one is converted")

        r = self.run_checker(base=base)
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn("core/A.h", r.stdout)
        # Prove the count really was flat, so this is not passing for the
        # ordinary shrink reason the other cases already cover.
        self.assertIn("1 converted on the base, 1 here", r.stdout)

    def test_deleting_the_row_exits_1(self):
        self.write_status({"core/A.h": "converted:#4099"})
        base = self.commit("base")
        self.write_status({})
        self.commit("head: row deleted")
        r = self.run_checker(base=base)
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn("row deleted", r.stdout)

    def test_deleting_the_whole_ledger_exits_1(self):
        self.write_status({"core/A.h": "converted:#4099"})
        base = self.commit("base")
        (self.root / STATUS_REL).unlink()
        r = self.run_checker(base=base)
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn("missing", r.stdout)

    def test_growing_exits_0(self):
        """The positive control, and it must not be vacuous.

        This asserts the base side was really read from git: the base
        commit has ONE conversion, the head has TWO, and the reported
        transition must say 1 -> 2. A checker that read the working tree
        for both sides would report 2 -> 2 and still exit 0.
        """
        self.write_status({"core/A.h": "converted:#4099",
                           "core/B.h": "unconverted"})
        base = self.commit("base: one conversion")

        self.write_status({"core/A.h": "converted:#4099",
                           "core/B.h": "converted:#4100"})
        self.commit("head: two conversions")

        r = self.run_checker(base=base)
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn("+1", r.stdout)
        self.assertIn("core/B.h", r.stdout)
        self.assertIn("1 → 2 converted", r.stdout)

    def test_flat_exits_0(self):
        self.write_status({"core/A.h": "converted:#4099"})
        base = self.commit("base")
        self.git("commit", "-q", "--allow-empty", "-m", "head: unrelated")
        r = self.run_checker(base=base)
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn("unchanged", r.stdout)

    def test_absent_base_ledger_is_not_a_violation(self):
        """The state this lands in: nothing to drop before the file exists."""
        self.git("commit", "-q", "--allow-empty", "-m", "base: no ledger yet")
        base = self.git("rev-parse", "HEAD").strip()
        self.write_status({"core/A.h": "unconverted"})
        self.commit("head: ledger introduced")
        r = self.run_checker(base=base)
        self.assertEqual(r.returncode, 0, r.stdout)

    def test_malformed_value_exits_1(self):
        self.write_status({"core/A.h": "Converted:#4099"})
        base = self.commit("base")
        r = self.run_checker(base=base)
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn("core/A.h", r.stdout)

    def test_unresolvable_base_exits_2_not_0(self):
        """Could-not-run must never be reported as passed."""
        self.write_status({"core/A.h": "converted:#4099"})
        self.commit("base")
        r = self.run_checker(base="no/such/ref")
        self.assertEqual(r.returncode, 2, r.stdout)
        self.assertIn("did NOT run", r.stdout)


class LiveLedgerTest(unittest.TestCase):
    """The committed ledger in THIS repo must itself be well-formed."""

    def test_every_live_touchpoint_has_a_row_and_a_legal_value(self):
        status = json.loads((REPO / STATUS_REL).read_text())
        for key, value in status.items():
            self.assertIsNotNone(checker.STATUS_RE.fullmatch(value),
                                 f"{key} -> {value!r}")
        gen = load(GEN_PATH, "gen_touchpoint_manifest")
        live = {f"{m}/{n}" for (m, n) in gen.scan()}
        missing = sorted(live - set(status))
        self.assertEqual(missing, [], "live touchpoints with no ledger row")


if __name__ == "__main__":
    unittest.main()
