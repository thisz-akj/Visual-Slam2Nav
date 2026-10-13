"""Minimal test runner (no pytest needed):  python check.py <lab number>  [-v]

Runs every test_* function in tests/test_NN_*.py and reports PASS / FAIL / TODO.
"""
import importlib.util
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: python check.py <lab> [-v]")
    lab, verbose = int(sys.argv[1]), "-v" in sys.argv
    files = sorted((ROOT / "tests").glob(f"test_{lab:02d}_*.py"))
    if not files:
        sys.exit(f"no tests for lab {lab}")
    counts = {"PASS": 0, "FAIL": 0, "TODO": 0}
    for path in files:
        spec = importlib.util.spec_from_file_location(path.stem, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        for name in [n for n in dir(mod) if n.startswith("test_")]:
            try:
                getattr(mod, name)()
                status, msg = "PASS", ""
            except NotImplementedError:
                status, msg = "TODO", ""
            except Exception as e:
                status = "FAIL"
                msg = traceback.format_exc() if verbose else f"{type(e).__name__}: {e}".strip()
            counts[status] += 1
            mark = {"PASS": "\033[32mPASS\033[0m", "FAIL": "\033[31mFAIL\033[0m", "TODO": "\033[33mTODO\033[0m"}
            print(f"  {mark[status]}  {name}" + (f"\n        {msg}" if msg else ""))
    print(f"\nLab {lab}: {counts['PASS']} passed, {counts['FAIL']} failed, {counts['TODO']} not implemented")
    sys.exit(0 if counts["FAIL"] == counts["TODO"] == 0 else 1)


if __name__ == "__main__":
    main()
