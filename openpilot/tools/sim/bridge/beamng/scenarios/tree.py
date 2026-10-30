"""The code a scenario invocation drove on: <run_dir>/tree.json, written at its start and checked again at its end.

initData.dirty is set on every sim run this Mac makes (common/version.is_dirty is true on a branch with no upstream, which
is every tp/* branch and detached worktree, and the main checkout keeps the documented msgq patch uncommitted:
tools/turnpilot/README.md), so the rlogs can't say whether a run's code had uncommitted edits. tree.json says it instead:

  schema            1
  stack_dir         the checkout the stack was launched from: the working directory of the running openpilot manager whose
                    OPENPILOT_PREFIX is the runner's (stack_dir_source 'manager'), else --stack-dir ('option'); null when
                    neither names one (then nothing below describes the stack, and clean is false)
  head, tree        its HEAD and HEAD^{tree}; openpilot_tree HEAD:openpilot (null without one)
  tracked_changes   git status --porcelain=v1 --untracked-files=no --ignore-submodules=all, one line each
  untracked_py      untracked *.py, *.capnp, *.h and *.cc outside the gitignored build output (git ls-files --others
                    --exclude-standard)
  submodules        each gitlink in HEAD: {path, gitlink (the sha HEAD records), head (the sha checked out), tracked_changes
                    (its git status lines), patch_ok (msgq_repo only: its diff is exactly the documented MSGQ_PATCH; null
                    for the others), dir (where it was read)}. A submodule the stack's checkout has no repo for (a worktree's
                    empty submodule dirs) is read where the stack imports it from: its package (SUBMODULE_PACKAGES) as the
                    venv resolves it with the checkout first on the path, the main checkout's for every worktree here
  clean             tracked_changes and untracked_py empty, and every submodule at its gitlink with no tracked changes but
                    msgq_repo's diff being exactly MSGQ_PATCH
  changed_during_run  null until the end check (finish) has read the checkout again (a run that never reached it, or
                    whose checkout couldn't be read then, is unverified), then whether anything differed; at_end holds the
                    end's snapshot when it did
  written_at        unix s of the last write; started_at, end_checked_at the start's and the end check's
  stack_git_commit  the manager's GitCommit param (manager_init), and problems: why the stack or its tree is in doubt

A run counts as code X only if its tree.json is clean, unchanged during the run, its head's openpilot/ tree and
opendbc_repo gitlink are X's, and its rlogs' initData.gitCommit is head (the demo page's rule; initData.dirty is ignored).
"""
import hashlib
import json
import os
import subprocess
import sys
import time

SCHEMA = 1
TREE_JSON = 'tree.json'
SOURCE_SUFFIXES = ('.py', '.capnp', '.h', '.cc')
MSGQ_PATH = 'msgq_repo'
MSGQ_PATCH = 'openpilot/tools/turnpilot/msgq-macos-poll.patch'
SUBMODULE_PACKAGES = {'msgq_repo': 'msgq', 'opendbc_repo': 'opendbc', 'panda': 'panda', 'rednose_repo': 'rednose',
                      'teleoprtc_repo': 'teleoprtc', 'tinygrad_repo': 'tinygrad'}
MANAGER = 'openpilot/system/manager/manager.py'
# the diff MSGQ_PATCH is compared with, against HEAD (staged or not), whatever the user's git config: `git diff`'s own
# output for the patch applied and left unstaged, as tools/turnpilot/README.md has it
DIFF = ('-c', 'color.ui=never', '-c', 'diff.algorithm=myers', 'diff', '--no-color', '--no-ext-diff', '--no-renames',
        '--src-prefix=a/', '--dst-prefix=b/', '-U3')
COMPARED = ('stack_dir', 'head', 'tree', 'openpilot_tree', 'tracked_changes', 'untracked_py', 'submodules', 'digest')


def git(cwd: str, *args: str, binary: bool = False):
  out = subprocess.run(['git', '-C', cwd, *args], capture_output=True, check=True).stdout
  return out if binary else out.decode()


def toplevel(path: str) -> str | None:
  """The git checkout path is in (its own when it is one), None outside any."""
  try:
    return os.path.realpath(git(path, 'rev-parse', '--show-toplevel').strip())
  except (subprocess.CalledProcessError, OSError):
    return None


def refresh(cwd: str):
  """Touched files read as modified until the index is refreshed (common/version.is_dirty does the same)."""
  subprocess.run(['git', '-C', cwd, 'update-index', '-q', '--refresh'], capture_output=True)


def status_lines(cwd: str, *extra: str) -> list[str]:
  out = git(cwd, 'status', '--porcelain=v1', '--untracked-files=no', *extra)
  return [ln for ln in out.splitlines() if ln.strip()]


def package_dir(stack_dir: str, package: str) -> str | None:
  """Where the stack imports package from: the venv's resolution with the checkout first on the path, as
  launch_openpilot.sh's PYTHONPATH puts it (the venv's .pth and editable finders after it)."""
  code = "\n".join(["import importlib.util, sys", "sys.path.insert(0, sys.argv[1])", "s = importlib.util.find_spec(sys.argv[2])",
                     "print((list(s.submodule_search_locations or []) or [s.origin or ''])[0] if s else '')"])
  try:
    out = subprocess.run([sys.executable, '-I', '-c', code, stack_dir, package], capture_output=True, text=True, timeout=30,
                         cwd=stack_dir).stdout.strip()
  except (OSError, subprocess.TimeoutExpired):
    return None
  return out or None


def submodule_dir(stack_dir: str, path: str, resolve=None) -> str | None:
  """The repo the stack runs a submodule's code from: its own checkout in stack_dir, else where its package resolves."""
  d = os.path.realpath(os.path.join(stack_dir, path))
  if os.path.isdir(d) and toplevel(d) == d:
    return d
  pkg = SUBMODULE_PACKAGES.get(path)
  loc = (resolve or package_dir)(stack_dir, pkg) if pkg else None
  top = toplevel(os.path.dirname(loc) if loc and os.path.isfile(loc) else loc) if loc else None
  return top if top is not None and top != toplevel(stack_dir) else None


def snapshot(stack_dir: str, resolve=None) -> dict:
  """The tree fields of stack_dir (module docstring), and a digest of its uncommitted content for the end check."""
  refresh(stack_dir)
  h = hashlib.sha256()
  head = git(stack_dir, 'rev-parse', 'HEAD').strip()
  tree = git(stack_dir, 'rev-parse', 'HEAD^{tree}').strip()
  try:
    op_tree = git(stack_dir, 'rev-parse', '--verify', '-q', 'HEAD:openpilot').strip() or None
  except subprocess.CalledProcessError:
    op_tree = None
  tracked = status_lines(stack_dir, '--ignore-submodules=all')
  h.update(git(stack_dir, *DIFF, '--binary', '--ignore-submodules=all', 'HEAD', binary=True))
  others = git(stack_dir, 'ls-files', '--others', '--exclude-standard', '-z').split('\0')
  untracked = sorted(p for p in others if p.endswith(SOURCE_SUFFIXES))
  for p in untracked:
    h.update(p.encode() + b'\0')
    try:
      with open(os.path.join(stack_dir, p), 'rb') as f:
        h.update(f.read())
    except OSError:
      pass
  try:
    with open(os.path.join(stack_dir, MSGQ_PATCH), 'rb') as f:
      patch = f.read()
  except OSError:
    patch = None
  subs = []
  for entry in git(stack_dir, 'ls-tree', '-r', '-z', 'HEAD').split('\0'):
    meta, _, path = entry.partition('\t')
    if not meta.startswith('160000 '):
      continue
    gitlink = meta.split()[2]
    d = submodule_dir(stack_dir, path, resolve)
    sub = {'path': path, 'gitlink': gitlink, 'head': None, 'tracked_changes': [], 'patch_ok': None, 'dir': d}
    if d is not None:
      refresh(d)
      sub['head'] = git(d, 'rev-parse', 'HEAD').strip()
      sub['tracked_changes'] = status_lines(d)
      diff = git(d, *DIFF, 'HEAD', binary=True)
      h.update(path.encode() + b'\0' + diff)
      if path == MSGQ_PATH:
        sub['patch_ok'] = patch is not None and diff == patch
    elif path == MSGQ_PATH:
      sub['patch_ok'] = False
    subs.append(sub)
  clean = not tracked and not untracked and all(
    s['head'] == s['gitlink'] and (not s['tracked_changes'] or (s['path'] == MSGQ_PATH and s['patch_ok'])) for s in subs)
  return {'stack_dir': os.path.realpath(stack_dir), 'head': head, 'tree': tree, 'openpilot_tree': op_tree, 'tracked_changes': tracked,
          'untracked_py': untracked, 'submodules': subs, 'clean': clean, 'digest': h.hexdigest()}


def manager_dirs(prefix: str, run=subprocess.run) -> list[str]:
  """The working directories of the running openpilot managers (a manager.py) whose OPENPILOT_PREFIX is prefix ('' for
  none): Linux from /proc, macOS from ps and lsof."""
  out = []
  if os.path.isdir('/proc/self'):
    for pid in os.listdir('/proc'):
      if not pid.isdigit():
        continue
      try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
          if not any(a.endswith(b'manager.py') for a in f.read().split(b'\0')):
            continue
        with open(f'/proc/{pid}/environ', 'rb') as f:
          env = dict(kv.split(b'=', 1) for kv in f.read().split(b'\0') if b'=' in kv)
        if env.get(b'OPENPILOT_PREFIX', b'').decode() == prefix:
          out.append(os.readlink(f'/proc/{pid}/cwd'))
      except OSError:
        continue
    return out
  ps = run(['ps', '-axww', '-o', 'pid=,command='], capture_output=True, text=True).stdout
  for line in ps.splitlines():
    pid, _, cmd = line.strip().partition(' ')
    if not pid.isdigit() or not any(a.endswith('manager.py') for a in cmd.split()):
      continue
    env = run(['ps', '-wwE', '-p', pid, '-o', 'command='], capture_output=True, text=True).stdout.split()
    if next((a.partition('=')[2] for a in env if a.startswith('OPENPILOT_PREFIX=')), '') != prefix:
      continue
    cwd = run(['lsof', '-a', '-p', pid, '-d', 'cwd', '-Fn'], capture_output=True, text=True).stdout
    out += [ln[1:] for ln in cwd.splitlines() if ln.startswith('n')]
  return out


def find_stack(prefix: str, option: str = '', managers=None) -> tuple[str | None, str, list[str]]:
  """(the stack's checkout, how it was found, problems): the running manager's for prefix (manager_dirs), else option."""
  dirs = manager_dirs(prefix) if managers is None else managers
  tops = sorted({t for t in (toplevel(d) for d in dirs) if t is not None and os.path.exists(os.path.join(t, MANAGER))})
  problems = []
  if len(tops) == 1:
    if option and os.path.realpath(option) != tops[0]:
      problems.append(f"--stack-dir {option} is not the running manager's checkout {tops[0]}: the manager's is recorded")
    return tops[0], 'manager', problems
  if len(tops) > 1:
    return None, 'none', [f"{len(tops)} openpilot managers run with OPENPILOT_PREFIX={prefix!r} ({', '.join(tops)}): which is the stack's isn't known"]
  if option:
    top = toplevel(option)
    if top is None:
      return None, 'none', [f"--stack-dir {option} is not a git checkout"]
    return top, 'option', [f"no running openpilot manager found with OPENPILOT_PREFIX={prefix!r}: --stack-dir taken as the stack's"]
  return None, 'none', [f"no running openpilot manager found with OPENPILOT_PREFIX={prefix!r}: the stack's checkout is unknown (--stack-dir)"]


def write(run_dir: str, doc: dict):
  doc['written_at'] = round(time.time(), 3)  # noqa: TID251  unix s, for the page
  tmp = os.path.join(run_dir, TREE_JSON + '.tmp')
  with open(tmp, 'w') as f:
    json.dump(doc, f, indent=1)
  os.replace(tmp, os.path.join(run_dir, TREE_JSON))


def start(run_dir: str, prefix: str, option: str = '', git_commit=None, managers=None, resolve=None) -> dict:
  """tree.json at the invocation's start (module docstring): written, and returned for finish()."""
  stack_dir, source, problems = find_stack(prefix, option, managers)
  doc: dict = {'schema': SCHEMA, 'stack_dir': None, 'stack_dir_source': source, 'head': None, 'tree': None, 'openpilot_tree': None,
               'tracked_changes': [], 'untracked_py': [], 'submodules': [], 'clean': False}
  if stack_dir is not None:
    try:
      doc.update(snapshot(stack_dir, resolve))
    except (subprocess.CalledProcessError, OSError) as e:
      problems.append(f"the stack's checkout {stack_dir} could not be read: {e!r}")
  if isinstance(git_commit, bytes):
    git_commit = git_commit.decode(errors='replace')
  doc['stack_git_commit'] = git_commit
  if doc['head'] is not None and git_commit != doc['head']:
    problems.append(f"the manager's GitCommit param {git_commit!r} is not the stack checkout's HEAD {doc['head']}")
  doc.update(changed_during_run=None, started_at=round(time.time(), 3), end_checked_at=None, problems=problems)  # noqa: TID251
  write(run_dir, doc)
  for p in problems:
    print(f"tree.json: {p}", flush=True)
  return doc


def finish(run_dir: str, doc: dict, resolve=None) -> dict:
  """The end check: the stack's checkout read again, changed_during_run from whether anything COMPARED differs (left
  null, unverified, when there is no checkout to read or it can't be read)."""
  end = None
  if doc.get('stack_dir') is not None:
    try:
      end = snapshot(doc['stack_dir'], resolve)
    except (subprocess.CalledProcessError, OSError) as e:
      doc['problems'] = doc.get('problems', []) + [f"the end check could not read {doc['stack_dir']}: {e!r}"]
  changed = None if end is None else any(end[k] != doc.get(k) for k in COMPARED)
  doc.update(changed_during_run=changed, end_checked_at=round(time.time(), 3))  # noqa: TID251
  if changed:
    doc['at_end'] = end
  write(run_dir, doc)
  if changed is not False:
    print(f"tree.json: the stack's checkout {'changed during the run' if changed else 'was not checked at the end'} " +
          f"({doc.get('stack_dir')})", flush=True)
  return doc
