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
                    for the others), dir (the repo read), own_dir (the checkout's own repo at path; null for a worktree's
                    empty submodule dir)}. Each is read where the stack imports it from, never just because the checkout
                    has a repo of its own there: its package (SUBMODULE_PACKAGES) as the stack's interpreter finds it, the
                    manager's dir and PYTHONPATH first (submodule_repos). That is the main checkout's for every checkout
                    here, a populated worktree submodule too: launch_openpilot.sh puts only the checkout root on PYTHONPATH,
                    which has no package dirs, so the venv's editable finders and _openpilot.pth give them. A checkout whose
                    own repo at path is not the one the stack imports, or a package found nowhere or outside any submodule
                    repo, is a problem
  clean             tracked_changes, untracked_py and the submodules' problems empty, and every submodule at its gitlink with
                    no tracked changes but msgq_repo's diff being exactly MSGQ_PATCH
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


class ResolveError(Exception):
  """Where the stack imports its packages from couldn't be found out."""


# run by the stack's interpreter (package_dirs): argv[1] the manager script's dir ('' for none) first on the path, as
# python puts ./manager.py's, then each package of argv[2:] found as the stack finds it; the last line is {name: where}
RESOLVE = "\n".join([
  "import importlib.util, json, sys",
  "sys.path[:0] = [sys.argv[1]] if sys.argv[1] else []",
  "def where(name):",
  "  try:",
  "    s = importlib.util.find_spec(name)",
  "  except Exception:",
  "    return None",
  "  return ((list(s.submodule_search_locations or []) or [s.origin or ''])[0] or None) if s else None",
  "print('\\n' + json.dumps({n: where(n) for n in sys.argv[2:]}))"])


def assumed_launch(stack_dir: str) -> dict:
  """How launch_openpilot.sh starts the stack from stack_dir, for a stack whose manager isn't found running (--stack-dir):
  this interpreter, in this environment with the checkout first on PYTHONPATH (${PYTHONPATH:+:$PYTHONPATH}), the
  manager's dir first on its path."""
  env = dict(os.environ)
  env['PYTHONPATH'] = stack_dir + (os.pathsep + env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
  mdir = os.path.join(stack_dir, os.path.dirname(MANAGER))
  return {'from': 'assumed', 'python': sys.executable, 'cwd': stack_dir, 'script_dir': mdir, 'env': env}


def package_dirs(launch: dict, packages: list[str]) -> dict[str, str | None]:
  """{package: where the stack imports it from (its dir, or its file for a module), None for nowhere}, as launch's
  interpreter finds it in launch's environment and working directory (its PYTHONPATH, then the venv's site, .pth files
  and editable finders), with the manager script's dir first. ResolveError when that can't be run."""
  if not packages:
    return {}
  if not launch.get('python'):
    raise ResolveError("the stack's interpreter is unknown" + (f" ({launch['why']})" if launch.get('why') else ''))
  first = '' if launch['env'].get('PYTHONSAFEPATH') else launch.get('script_dir') or ''
  try:
    out = subprocess.run([launch['python'], '-P', '-c', RESOLVE, first, *packages], capture_output=True, text=True, timeout=60,
                         env=launch['env'], cwd=launch['cwd'], stdin=subprocess.DEVNULL)
  except (OSError, subprocess.TimeoutExpired) as e:
    raise ResolveError(f"the stack's interpreter {launch['python']} could not be run: {e!r}") from e
  try:
    found = json.loads(out.stdout.rstrip().rpartition('\n')[2]) if out.returncode == 0 else None
  except ValueError:
    found = None
  if not isinstance(found, dict):
    raise ResolveError(f"the stack's interpreter {launch['python']} could not resolve its packages (exit {out.returncode}): " +
                       out.stderr.strip()[-300:])
  return {p: found.get(p) for p in packages}


def submodule_repos(stack_dir: str, paths: list[str], launch: dict | None = None, resolve=None) -> tuple[dict, list[str]]:
  """({path: (the repo the stack runs path's code from, the checkout's own repo at path, why they are in doubt)}, the
  problems resolving them). A submodule with a package (SUBMODULE_PACKAGES) is read where the stack imports that
  package from (package_dirs, as launch starts the stack; launch_openpilot.sh's when None), whatever the checkout's own
  folder holds: the stack's path never has the checkout's own submodule repos on it here (launch_openpilot.sh puts
  the checkout root first, which has no package dirs, and the venv's editable finders and _openpilot.pth give the main
  checkout's). The checkout's own repo there (None for a worktree's empty submodule dir) is a cross-check: one that
  isn't the repo the stack imports is in doubt. A gitlink with no package is read in its own repo."""
  launch = launch or assumed_launch(stack_dir)
  self_top = toplevel(stack_dir)
  pkgs = [SUBMODULE_PACKAGES[p] for p in paths if p in SUBMODULE_PACKAGES]
  problems = []
  try:
    locs = (resolve or package_dirs)(launch, pkgs) if pkgs else {}
  except ResolveError as e:
    problems.append(str(e))
    locs = None
  out = {}
  for path in paths:
    d = os.path.realpath(os.path.join(stack_dir, path))
    own = d if os.path.isdir(d) and toplevel(d) == d else None
    pkg = SUBMODULE_PACKAGES.get(path)
    if pkg is None:
      out[path] = (own, own, None)
      continue
    loc = None if locs is None else locs.get(pkg)
    top = toplevel(os.path.dirname(loc) if os.path.isfile(loc) else loc) if loc else None
    why = None
    if locs is None:
      why = f"where the stack imports {pkg} from is unknown"
    elif loc is None:
      why = f"the stack's interpreter finds no {pkg} package"
    elif top is None or top == self_top:
      why, top = f"the stack imports {pkg} from {loc}, which is in no submodule checkout", None
    elif own is not None and own != top:
      short = {r: git(r, 'rev-parse', '--short', 'HEAD').strip() for r in (top, own)}
      why = f"the stack imports {pkg} from {top} ({short[top]}), not from this checkout's own {path} ({own}, {short[own]})"
    out[path] = (top, own, why)
  return out, problems


def submodule_dir(stack_dir: str, path: str, launch: dict | None = None, resolve=None) -> str | None:
  """The repo the stack runs a submodule's code from (submodule_repos)."""
  return submodule_repos(stack_dir, [path], launch, resolve)[0][path][0]


def snapshot(stack_dir: str, resolve=None, launch: dict | None = None) -> dict:
  """The tree fields of stack_dir (module docstring), a digest of its uncommitted content for the end check, and the
  problems that put its submodules in doubt (submodule_repos; they make it unclean)."""
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
  links = [(e.partition('\t')[2], e.split()[2]) for e in git(stack_dir, 'ls-tree', '-r', '-z', 'HEAD').split('\0') if e.startswith('160000 ')]
  repos, problems = submodule_repos(stack_dir, [path for path, _ in links], launch, resolve)
  subs = []
  for path, gitlink in links:
    d, own, why = repos[path]
    if why is not None:
      problems.append(f"{path}: {why}")
    sub = {'path': path, 'gitlink': gitlink, 'head': None, 'tracked_changes': [], 'patch_ok': None, 'dir': d, 'own_dir': own}
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
  clean = not tracked and not untracked and not problems and all(
    s['head'] == s['gitlink'] and (not s['tracked_changes'] or (s['path'] == MSGQ_PATH and s['patch_ok'])) for s in subs)
  return {'stack_dir': os.path.realpath(stack_dir), 'head': head, 'tree': tree, 'openpilot_tree': op_tree, 'tracked_changes': tracked,
          'untracked_py': untracked, 'submodules': subs, 'clean': clean, 'digest': h.hexdigest(), 'problems': problems}


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
      snap = snapshot(stack_dir, resolve)
      problems += snap.pop('problems')
      doc.update(snap)
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
      new = [p for p in end.pop('problems') if p not in doc.get('problems', [])]
      if new:
        doc['problems'] = doc.get('problems', []) + [f"at the end check, {p}" for p in new]
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
