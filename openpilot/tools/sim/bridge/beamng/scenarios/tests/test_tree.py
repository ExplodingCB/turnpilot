"""tree.json (scenarios/tree.py): what code a scenario invocation drove on, since initData.dirty is set on every sim run
here. Each test builds a checkout in tmp_path: openpilot/ with a source file, msgq_repo and opendbc_repo as gitlinks with
their own repos, and the documented msgq patch committed and applied, as the main checkout keeps it. Where the stack
imports msgq and opendbc from is faked (own_packages: the checkout's own repos) unless a test resolves it for real."""
import json
import os
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from openpilot.tools.sim.bridge.beamng.scenarios import tree

MSGQ_CC = "int poll_ms = std::min(ms, 10);\nint other = 1;\n"
PATCHED = "int poll_ms = std::min(ms, 1);\nint other = 1;\n"


def git(cwd, *args):
  return subprocess.run(['git', '-C', str(cwd), '-c', 'user.name=t', '-c', 'user.email=t@t', *args], capture_output=True, text=True,
                        check=True).stdout.strip()


def write(path, text):
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, 'w') as f:
    f.write(text)


def repo(path, files):
  for rel, text in files.items():
    write(os.path.join(path, rel), text)
  git(path, 'init', '-q')
  git(path, 'add', '-A')
  git(path, 'commit', '-q', '-m', 'init')
  return git(path, 'rev-parse', 'HEAD')


@pytest.fixture(autouse=True)
def plain_git(monkeypatch, tmp_path):
  """No user or system git config: the checkouts here are the test's own."""
  monkeypatch.setenv('HOME', str(tmp_path))
  monkeypatch.setenv('GIT_CONFIG_NOSYSTEM', '1')
  monkeypatch.setenv('GIT_CONFIG_GLOBAL', os.devnull)


@pytest.fixture
def checkout(tmp_path):
  """The superproject: committed with gitlinks to msgq_repo and opendbc_repo at their heads, the msgq patch committed in
  openpilot/tools/turnpilot and applied, unstaged, in msgq_repo."""
  root = tmp_path / 'tp'
  root.mkdir()
  msgq = repo(root / 'msgq_repo', {'msgq/msgq.cc': MSGQ_CC, 'msgq/__init__.py': ''})
  odbc = repo(root / 'opendbc_repo', {'opendbc/__init__.py': ''})
  write(root / 'msgq_repo/msgq/msgq.cc', PATCHED)
  patch = subprocess.run(['git', '-C', str(root / 'msgq_repo'), 'diff'], capture_output=True, text=True, check=True).stdout
  write(root / tree.MSGQ_PATCH, patch)
  write(root / 'openpilot/selfdrive/x.py', "X = 1\n")
  write(root / tree.MANAGER, "")
  write(root / '.gitignore', "build/\n*.so\n")
  git(root, 'init', '-q')
  git(root, 'add', '.gitignore', 'openpilot')
  git(root, 'update-index', '--add', '--cacheinfo', f'160000,{msgq},msgq_repo')
  git(root, 'update-index', '--add', '--cacheinfo', f'160000,{odbc},opendbc_repo')
  git(root, 'commit', '-q', '-m', 'superproject')
  return root


REPOS = {'msgq': 'msgq_repo', 'opendbc': 'opendbc_repo'}


def no_packages(launch, packages):
  return dict.fromkeys(packages)


def own_packages(root):
  """The stack imports the checkout's own submodule repos (as upstream's layout links them at the root)."""
  return lambda launch, packages: {p: str(root / REPOS[p] / p) for p in packages}


def snap(root):
  return tree.snapshot(str(root), resolve=own_packages(root))


class TestSnapshot:
  def test_a_clean_tree_with_the_documented_msgq_patch(self, checkout):
    s = snap(checkout)
    assert s['clean'] and s['tracked_changes'] == [] and s['untracked_py'] == []
    assert s['head'] == git(checkout, 'rev-parse', 'HEAD') and s['tree'] == git(checkout, 'rev-parse', 'HEAD^{tree}')
    assert s['openpilot_tree'] == git(checkout, 'rev-parse', 'HEAD:openpilot')
    subs = {x['path']: x for x in s['submodules']}
    assert set(subs) == {'msgq_repo', 'opendbc_repo'}
    assert subs['msgq_repo']['patch_ok'] is True and subs['msgq_repo']['tracked_changes'] == [' M msgq/msgq.cc']
    assert subs['opendbc_repo']['patch_ok'] is None and subs['opendbc_repo']['tracked_changes'] == []
    assert all(x['head'] == x['gitlink'] for x in subs.values())

  def test_a_tracked_edit_isnt_clean(self, checkout):
    write(checkout / 'openpilot/selfdrive/x.py', "X = 2\n")
    s = snap(checkout)
    assert not s['clean'] and s['tracked_changes'] == [' M openpilot/selfdrive/x.py']

  def test_an_untracked_source_file_isnt_clean_but_build_output_and_other_files_are(self, checkout):
    write(checkout / 'build/gen.py', "")  # gitignored build output
    write(checkout / 'openpilot/notes.txt', "")
    write(checkout / 'openpilot/x.so', "")
    assert snap(checkout)['clean']
    for rel in ('openpilot/selfdrive/new.py', 'openpilot/cereal/new.capnp', 'openpilot/common/new.h', 'openpilot/common/new.cc'):
      write(checkout / rel, "")
    s = snap(checkout)
    assert not s['clean']
    assert s['untracked_py'] == ['openpilot/cereal/new.capnp', 'openpilot/common/new.cc', 'openpilot/common/new.h', 'openpilot/selfdrive/new.py']

  def test_a_submodule_off_its_gitlink_isnt_clean(self, checkout):
    write(checkout / 'opendbc_repo/opendbc/car.py', "")
    git(checkout / 'opendbc_repo', 'add', '-A')
    git(checkout / 'opendbc_repo', 'commit', '-q', '-m', 'local')
    s = snap(checkout)
    sub = next(x for x in s['submodules'] if x['path'] == 'opendbc_repo')
    assert not s['clean'] and sub['head'] != sub['gitlink'] and sub['tracked_changes'] == []

  def test_msgq_with_another_diff_isnt_clean(self, checkout):
    write(checkout / 'msgq_repo/msgq/msgq.cc', "int poll_ms = std::min(ms, 2);\nint other = 1;\n")
    s = snap(checkout)
    msgq = next(x for x in s['submodules'] if x['path'] == 'msgq_repo')
    assert not s['clean'] and msgq['patch_ok'] is False
    # the patch and more, in another file
    write(checkout / 'msgq_repo/msgq/msgq.cc', PATCHED)
    write(checkout / 'msgq_repo/msgq/__init__.py', "import os\n")
    assert not snap(checkout)['clean']
    # unpatched: no tracked changes, so clean, but not the patch
    write(checkout / 'msgq_repo/msgq/__init__.py', "")
    write(checkout / 'msgq_repo/msgq/msgq.cc', MSGQ_CC)
    s = snap(checkout)
    assert s['clean'] and next(x for x in s['submodules'] if x['path'] == 'msgq_repo')['patch_ok'] is False

  def test_a_tracked_edit_in_another_submodule_isnt_clean(self, checkout):
    write(checkout / 'opendbc_repo/opendbc/__init__.py', "X = 1\n")
    assert not snap(checkout)['clean']

  def test_a_worktrees_empty_submodule_dir_is_read_where_its_package_resolves(self, checkout, tmp_path):
    # a worktree has empty submodule dirs: the stack imports their packages from the main checkout (the venv's .pth)
    wt = tmp_path / 'wt'
    git(checkout, 'worktree', 'add', '-q', '--detach', str(wt))
    assert os.listdir(wt / 'msgq_repo') == []
    found = {'msgq': str(checkout / 'msgq_repo/msgq'), 'opendbc': str(checkout / 'opendbc_repo/opendbc/__init__.py')}
    s = tree.snapshot(str(wt), resolve=lambda launch, pkgs: {p: found.get(p) for p in pkgs})
    subs = {x['path']: x for x in s['submodules']}
    assert s['clean'] and subs['msgq_repo']['dir'] == os.path.realpath(checkout / 'msgq_repo') and subs['msgq_repo']['patch_ok']
    assert subs['opendbc_repo']['dir'] == os.path.realpath(checkout / 'opendbc_repo') and s['problems'] == []
    assert subs['msgq_repo']['own_dir'] is None and subs['opendbc_repo']['own_dir'] is None
    # one that resolves nowhere, or to the worktree itself, has no head: not clean, and said
    for where in (str(wt / 'msgq_repo'), None):
      s = tree.snapshot(str(wt), resolve=lambda launch, pkgs, where=where: {p: where if p == 'msgq' else found.get(p) for p in pkgs})
      assert not s['clean'] and next(x for x in s['submodules'] if x['path'] == 'msgq_repo')['head'] is None
      assert [p.split(':')[0] for p in s['problems']] == ['msgq_repo']

  def test_the_checkouts_own_submodule_repo_isnt_read_when_the_stack_imports_another(self, checkout, tmp_path):
    # tp-blinker's: its own opendbc_repo is checked out at its gitlink, but the stack imports opendbc from the main
    # checkout's (the venv's editable finder), at another commit
    main = tmp_path / 'main_opendbc'
    git(tmp_path, 'clone', '-q', str(checkout / 'opendbc_repo'), str(main))
    write(main / 'opendbc/car.py', "")
    git(main, 'add', '-A')
    git(main, 'commit', '-q', '-m', 'main')
    found = {'msgq': str(checkout / 'msgq_repo/msgq'), 'opendbc': str(main / 'opendbc')}
    s = tree.snapshot(str(checkout), resolve=lambda launch, pkgs: {p: found[p] for p in pkgs})
    sub = next(x for x in s['submodules'] if x['path'] == 'opendbc_repo')
    assert sub['dir'] == os.path.realpath(main) and sub['own_dir'] == os.path.realpath(checkout / 'opendbc_repo')
    assert sub['head'] == git(main, 'rev-parse', 'HEAD') != sub['gitlink'] and not s['clean']
    (problem,) = s['problems']
    assert problem.startswith('opendbc_repo: ') and os.path.realpath(main) in problem and os.path.realpath(checkout / 'opendbc_repo') in problem
    # the other repo at the gitlink: the code that runs is the gitlink's, but not the checkout's own repo: in doubt, not clean
    git(main, 'reset', '-q', '--hard', 'HEAD~1')
    s = tree.snapshot(str(checkout), resolve=lambda launch, pkgs: {p: found[p] for p in pkgs})
    sub = next(x for x in s['submodules'] if x['path'] == 'opendbc_repo')
    assert sub['head'] == sub['gitlink'] and sub['tracked_changes'] == [] and not s['clean'] and len(s['problems']) == 1
    # the stack imports the checkout's own: clean
    assert snap(checkout)['clean'] and snap(checkout)['problems'] == []

  def test_a_package_the_stacks_interpreter_cant_resolve_isnt_clean(self, checkout):
    def broken(launch, packages):
      raise tree.ResolveError("the stack's interpreter /nowhere/python3 could not be run")
    s = tree.snapshot(str(checkout), resolve=broken)
    assert not s['clean'] and all(x['dir'] is None and x['head'] is None for x in s['submodules'])
    assert s['problems'][0] == "the stack's interpreter /nowhere/python3 could not be run"
    assert sorted(p.split(':')[0] for p in s['problems'][1:]) == ['msgq_repo', 'opendbc_repo']

  def test_a_populated_submodule_the_stack_doesnt_import_isnt_clean(self, checkout, tmp_path, monkeypatch):
    # the reviewer's tree_repro.sh, resolved for real: the checkout's own opendbc_repo is a repo at its gitlink, but its
    # root has no opendbc/ (this repo's layout), so the stack (the checkout first on PYTHONPATH) imports opendbc from
    # the venv's editable finder, the main checkout's, or finds none: either way not the checkout's own
    monkeypatch.delenv('PYTHONPATH', raising=False)
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    doc = tree.finish(str(run_dir), tree.start(str(run_dir), 'tpsim', git_commit=git(checkout, 'rev-parse', 'HEAD'),
                                               managers=[str(checkout / 'openpilot/system/manager')]))
    on_disk = json.load(open(run_dir / 'tree.json'))
    sub = next(x for x in on_disk['submodules'] if x['path'] == 'opendbc_repo')
    assert not on_disk['clean'] and on_disk['changed_during_run'] is False and doc['clean'] is False
    assert sub['own_dir'] == os.path.realpath(checkout / 'opendbc_repo') and sub['dir'] != sub['own_dir']
    assert any(p.startswith('opendbc_repo: ') for p in on_disk['problems'])

  def test_the_stack_importing_its_own_submodules_is_clean_resolved_for_real(self, checkout, monkeypatch):
    # upstream's layout: msgq and opendbc linked at the root, which launch_openpilot.sh puts first on PYTHONPATH
    monkeypatch.delenv('PYTHONPATH', raising=False)
    os.symlink('msgq_repo/msgq', checkout / 'msgq')
    os.symlink('opendbc_repo/opendbc', checkout / 'opendbc')
    git(checkout, 'add', 'msgq', 'opendbc')
    git(checkout, 'commit', '-q', '-m', 'links')
    s = tree.snapshot(str(checkout))
    assert s['clean'] and s['problems'] == []
    assert {x['path']: x['dir'] for x in s['submodules']} == {p: os.path.realpath(checkout / p) for p in ('msgq_repo', 'opendbc_repo')}

  def test_another_opendbc_on_the_stacks_pythonpath_is_what_it_runs(self, checkout, tmp_path, monkeypatch):
    # launch_openpilot.sh keeps the PYTHONPATH it was started with after the checkout: an opendbc there is the stack's
    other = tmp_path / 'other'
    repo(other, {'opendbc/__init__.py': "X = 1\n"})
    os.symlink('msgq_repo/msgq', checkout / 'msgq')
    monkeypatch.setenv('PYTHONPATH', str(other))
    s = tree.snapshot(str(checkout))
    sub = next(x for x in s['submodules'] if x['path'] == 'opendbc_repo')
    assert sub['dir'] == os.path.realpath(other) and sub['head'] == git(other, 'rev-parse', 'HEAD') and not s['clean']
    assert [p.split(':')[0] for p in s['problems']] == ['opendbc_repo']


class TestResolution:
  def test_the_stacks_path_then_the_venvs(self, tmp_path, monkeypatch):
    a, b, m = tmp_path / 'a', tmp_path / 'b', tmp_path / 'm'
    for d in (a, b):
      write(d / 'zz_tree_probe_pkg/__init__.py', "")
    write(b / 'zz_tree_probe_mod.py', "")
    monkeypatch.setenv('PYTHONPATH', str(b))  # the runner's own path is not the stack's
    launch = {'python': sys.executable, 'cwd': str(tmp_path), 'script_dir': str(m), 'env': {'PYTHONPATH': f"{a}{os.pathsep}{b}"}}
    names = ['zz_tree_probe_pkg', 'zz_tree_probe_mod', 'zz_tree_probe_missing', 'pytest']
    found = tree.package_dirs(launch, names)
    assert found['zz_tree_probe_pkg'] == str(a / 'zz_tree_probe_pkg') and found['zz_tree_probe_mod'] == str(b / 'zz_tree_probe_mod.py')
    assert found['zz_tree_probe_missing'] is None and os.path.isdir(found['pytest'])  # the venv's site is still there
    # relative entries are the manager's cwd's, as python takes them
    launch['env'] = {'PYTHONPATH': f"b{os.pathsep}a"}
    assert tree.package_dirs(launch, names)['zz_tree_probe_pkg'] == str(b / 'zz_tree_probe_pkg')
    # the manager script's own dir is first, as for ./manager.py
    write(m / 'zz_tree_probe_pkg/__init__.py', "")
    assert tree.package_dirs(launch, names)['zz_tree_probe_pkg'] == str(m / 'zz_tree_probe_pkg')
    assert tree.package_dirs({**launch, 'env': {'PYTHONPATH': str(a)}}, []) == {}
    with pytest.raises(tree.ResolveError, match="could not be run"):
      tree.package_dirs({**launch, 'python': str(tmp_path / 'no_python')}, names)
    with pytest.raises(tree.ResolveError, match="interpreter is unknown"):
      tree.package_dirs({**launch, 'python': None}, names)

  def test_the_assumed_launch_is_launch_openpilot_shs(self, tmp_path, monkeypatch):
    # no manager found (--stack-dir): this interpreter, the checkout first on PYTHONPATH, then the one it was given
    monkeypatch.delenv('PYTHONPATH', raising=False)
    launch = tree.assumed_launch(str(tmp_path))
    assert launch['python'] == sys.executable and launch['env']['PYTHONPATH'] == str(tmp_path) and launch['from'] == 'assumed'
    assert launch['script_dir'] == str(tmp_path / 'openpilot/system/manager')
    other = tmp_path / 'other'
    write(other / 'zz_tree_probe_pkg/__init__.py', "")
    monkeypatch.setenv('PYTHONPATH', str(other))
    launch = tree.assumed_launch(str(tmp_path))
    assert launch['env']['PYTHONPATH'] == f"{tmp_path}{os.pathsep}{other}"
    assert tree.package_dirs(launch, ['zz_tree_probe_pkg']) == {'zz_tree_probe_pkg': str(other / 'zz_tree_probe_pkg')}

  def test_this_checkouts_msgq_diff_is_the_documented_patch(self, monkeypatch):
    # the real thing: the msgq_repo this checkout's stack runs (its own, or the main checkout's for a worktree) carries the
    # patch, and the comparison's diff reproduces it byte for byte
    monkeypatch.undo()  # the user's own git, as the runner reads it
    root = os.path.realpath(os.path.join(os.path.dirname(tree.__file__), *(['..'] * 6)))
    msgq = tree.submodule_dir(root, tree.MSGQ_PATH)
    if msgq is None or not tree.git(msgq, *tree.DIFF, 'HEAD', binary=True):
      pytest.skip("no msgq_repo with the patch applied here")
    with open(os.path.join(root, tree.MSGQ_PATCH), 'rb') as f:
      assert tree.git(msgq, *tree.DIFF, 'HEAD', binary=True) == f.read()


def manager_record(cwd, env, argv=('python3', './manager.py')):
  """A running manager as running_managers reads it: launched as launch_openpilot.sh does (./manager.py, whose shebang
  is env python3), this venv's bin first on its PATH."""
  return {'pid': 1, 'cwd': str(cwd), 'argv': list(argv), 'env': {'PATH': os.path.dirname(sys.executable), **env}}


VENV_PYTHON3 = os.path.join(os.path.dirname(sys.executable), 'python3')


class TestFindStack:
  def test_the_running_managers_checkout_is_the_stacks(self, checkout, tmp_path):
    cwd = str(checkout / 'openpilot/system/manager')
    d, source, problems, launch = tree.find_stack('tpsim', managers=[cwd])
    assert (d, source, problems, launch['from']) == (os.path.realpath(checkout), 'manager', [], 'assumed')  # a bare cwd
    d, source, problems, _ = tree.find_stack('tpsim', option=str(tmp_path), managers=[cwd])
    assert (d, source) == (os.path.realpath(checkout), 'manager') and 'not the running manager' in problems[0]

  @pytest.mark.skipif(not os.path.exists(VENV_PYTHON3), reason="no python3 beside this interpreter")
  def test_the_running_managers_launch_is_how_its_imports_are_resolved(self, checkout, tmp_path):
    cwd = checkout / 'openpilot/system/manager'
    d, source, problems, launch = tree.find_stack('tpsim', managers=[manager_record(cwd, {'PYTHONPATH': f"{checkout}{os.pathsep}/x y"})])
    assert (d, source, problems) == (os.path.realpath(checkout), 'manager', [])
    assert launch['from'] == 'manager' and launch['python'] == VENV_PYTHON3 and launch['cwd'] == str(cwd)
    assert launch['script_dir'] == os.path.realpath(cwd) and launch['env']['PYTHONPATH'] == f"{checkout}{os.pathsep}/x y"
    # python3 found nowhere on its PATH, or interpreter options this check doesn't model: its imports are unknown
    for record, why in ((manager_record(cwd, {'PATH': str(tmp_path)}), "'python3' is not on its PATH"),
                        (manager_record(cwd, {}, argv=('python3', '-E', './manager.py')), "interpreter options ['-E']")):
      launch = tree.find_stack('tpsim', managers=[record])[3]
      assert launch['python'] is None and why in launch['why']
      with pytest.raises(tree.ResolveError, match="interpreter is unknown"):
        tree.package_dirs(launch, ['msgq'])
    # two managers in it started differently: which one's imports are the stack's isn't known
    records = [manager_record(cwd, {'PYTHONPATH': str(checkout)}), manager_record(cwd, {'PYTHONPATH': f"/x{os.pathsep}{checkout}"})]
    _, _, problems, launch = tree.find_stack('tpsim', managers=records)
    assert launch['python'] is None and problems[0].startswith('2 openpilot managers run from')
    records[1]['env']['PYTHONPATH'] = str(checkout)
    records[1]['env']['_'] = 'another shell'  # what else differs in their environments doesn't change their imports
    assert tree.find_stack('tpsim', managers=records)[2:] == ([], tree.launch_of(records[0]))

  def test_no_manager_or_two_leave_it_unknown_unless_named(self, checkout, tmp_path):
    assert tree.find_stack('tpsim', managers=[])[:2] == (None, 'none')
    d, source, _, launch = tree.find_stack('tpsim', option=str(checkout), managers=[])
    assert (d, source, launch['from']) == (os.path.realpath(checkout), 'option', 'assumed')
    other = tmp_path / 'other'
    repo(other, {tree.MANAGER: ''})
    d, source, problems, launch = tree.find_stack('tpsim', managers=[str(checkout), str(other)])
    assert (d, source, launch) == (None, 'none', None) and problems[0].startswith('2 openpilot managers')

  def test_managers_found_by_ps_procargs_and_lsof(self, monkeypatch):
    monkeypatch.setattr(tree.os.path, 'isdir', lambda p: False if p == '/proc/self' else os.path.isdir(p))
    out = {('ps', '-axww'): "  101 python3 ./manager.py\n  102 python3 ./manager.py\n  103 vim manager.txt\n  104 python3 ./manager.py\n" +
                            "  105 python3 ./manager.py.bak\n",
           ('lsof', '101'): "p101\nfcwd\nn/a/tp/openpilot/system/manager\n", ('lsof', '102'): "p102\nfcwd\nn/b\n"}
    args = {101: (['python3', './manager.py'], {'PATH': '/bin', 'OPENPILOT_PREFIX': 'tpsim', 'PYTHONPATH': '/a b/c:/d'}),
            102: (['python3', './manager.py'], {'PATH': '/bin'}), 104: None,  # another user's: unreadable
            105: (['python3', './manager.py.bak'], {'OPENPILOT_PREFIX': 'tpsim'})}

    def run(cmd, **kw):
      return SimpleNamespace(stdout=out[(cmd[0], cmd[1]) if cmd[0] == 'ps' else (cmd[0], cmd[3])])
    assert tree.running_managers('tpsim', run=run, read_args=args.get) == [
      {'pid': 101, 'cwd': '/a/tp/openpilot/system/manager', 'argv': ['python3', './manager.py'], 'env': args[101][1]}]
    assert [m['cwd'] for m in tree.running_managers('', run=run, read_args=args.get)] == ['/b']

  @pytest.mark.skipif(sys.platform != 'darwin' and not os.path.isdir('/proc/self'), reason="no way to read a process's environment here")
  @pytest.mark.skipif(not os.path.exists(VENV_PYTHON3), reason="no python3 beside this interpreter")
  def test_a_live_managers_environment_is_read_exactly(self, checkout, tmp_path):
    # a stub manager.py launched as launch_openpilot.sh launches the real one (exec ./manager.py, env python3), its
    # PYTHONPATH holding a dir with a space in it (ps -wwE's output can't tell where such a value ends)
    spaced = tmp_path / 'with space'
    repo(spaced, {'zz_tree_probe_pkg/__init__.py': ""})
    mdir = checkout / 'openpilot/system/manager'
    write(mdir / 'manager.py', "#!/usr/bin/env python3\nimport time\ntime.sleep(60)\n")
    os.chmod(mdir / 'manager.py', 0o755)
    prefix = f"tptree{os.getpid()}"
    env = {'PATH': f"{os.path.dirname(sys.executable)}{os.pathsep}/usr/bin{os.pathsep}/bin", 'OPENPILOT_PREFIX': prefix,
           'PYTHONPATH': f"{checkout}{os.pathsep}{spaced}", 'HOME': str(tmp_path)}
    proc = subprocess.Popen(['./manager.py'], cwd=mdir, env=env)
    try:
      for _ in range(100):
        found = tree.running_managers(prefix)
        if found:
          break
        time.sleep(0.1)
      (m,) = found
      assert m['pid'] == proc.pid and os.path.realpath(m['cwd']) == os.path.realpath(mdir)
      assert m['argv'] == ['python3', './manager.py'] and m['env']['PYTHONPATH'] == env['PYTHONPATH']
      d, source, problems, launch = tree.find_stack(prefix)
      assert (d, source, problems, launch['python']) == (os.path.realpath(checkout), 'manager', [], VENV_PYTHON3)
      assert tree.package_dirs(launch, ['zz_tree_probe_pkg']) == {'zz_tree_probe_pkg': str(spaced / 'zz_tree_probe_pkg')}
    finally:
      proc.kill()
      proc.wait()


class TestTreeJson:
  def test_start_writes_it_unverified_and_finish_checks_it(self, checkout, tmp_path):
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    cwd = str(checkout / 'openpilot/system/manager')
    head = git(checkout, 'rev-parse', 'HEAD')
    doc = tree.start(str(run_dir), 'tpsim', git_commit=head, managers=[cwd], resolve=own_packages(checkout))
    on_disk = json.load(open(run_dir / 'tree.json'))
    assert on_disk['schema'] == 1 and on_disk['clean'] and on_disk['stack_dir'] == os.path.realpath(checkout)
    assert on_disk['changed_during_run'] is None and on_disk['end_checked_at'] is None  # unverified until the end check
    assert on_disk['problems'] == [] and on_disk['stack_git_commit'] == head
    for k in ('head', 'tree', 'openpilot_tree', 'tracked_changes', 'untracked_py', 'submodules', 'written_at'):
      assert k in on_disk
    tree.finish(str(run_dir), doc, resolve=own_packages(checkout))
    on_disk = json.load(open(run_dir / 'tree.json'))
    assert on_disk['changed_during_run'] is False and on_disk['end_checked_at'] is not None and 'at_end' not in on_disk

  @pytest.mark.skipif(not os.path.exists(VENV_PYTHON3), reason="no python3 beside this interpreter")
  def test_the_stacks_launch_is_recorded_and_the_end_check_resolves_it_again(self, checkout, tmp_path):
    other = tmp_path / 'other'
    repo(other, {'opendbc/__init__.py': "X = 1\n"})
    os.symlink('msgq_repo/msgq', checkout / 'msgq')
    record = manager_record(checkout / 'openpilot/system/manager', {'PYTHONPATH': f"{checkout}{os.pathsep}{other}", 'TP_SECRET': 'hunter2'})
    head = git(checkout, 'rev-parse', 'HEAD')
    doc = tree.start(str(tmp_path), 'tpsim', git_commit=head, managers=[record])
    text = open(tmp_path / 'tree.json').read()
    on_disk = json.loads(text)
    assert on_disk['stack_import'] == {'from': 'manager', 'python': VENV_PYTHON3, 'cwd': record['cwd'],
                                       'script_dir': os.path.realpath(record['cwd']), 'pythonpath': record['env']['PYTHONPATH']}
    assert 'hunter2' not in text and '_launch' not in on_disk  # the manager's environment is never written
    sub = next(x for x in on_disk['submodules'] if x['path'] == 'opendbc_repo')
    assert sub['dir'] == os.path.realpath(other) and not on_disk['clean'] and [p.split(':')[0] for p in on_disk['problems']] == ['opendbc_repo']
    # the end check resolves as the manager did, though it has exited (the runner's own environment has no other/)
    tree.finish(str(tmp_path), doc)
    on_disk = json.load(open(tmp_path / 'tree.json'))
    assert on_disk['changed_during_run'] is False and len(on_disk['problems']) == 1

  def test_an_edit_during_the_run_is_changed(self, checkout, tmp_path):
    write(checkout / 'openpilot/selfdrive/x.py', "X = 2\n")  # dirty from the start: the same status line at the end
    doc = tree.start(str(tmp_path), 'tpsim', managers=[str(checkout)], resolve=own_packages(checkout))
    write(checkout / 'openpilot/selfdrive/x.py', "X = 3\n")
    tree.finish(str(tmp_path), doc, resolve=own_packages(checkout))
    on_disk = json.load(open(tmp_path / 'tree.json'))
    assert on_disk['changed_during_run'] is True and not on_disk['clean'] and on_disk['at_end']['tracked_changes'] == on_disk['tracked_changes']

  def test_an_unknown_stack_is_written_unclean_with_why(self, tmp_path):
    tree.finish(str(tmp_path), tree.start(str(tmp_path), 'tpsim', managers=[], resolve=no_packages), resolve=no_packages)
    on_disk = json.load(open(tmp_path / 'tree.json'))
    assert on_disk['stack_dir'] is None and not on_disk['clean'] and on_disk['changed_during_run'] is None
    assert "no running openpilot manager found with OPENPILOT_PREFIX='tpsim'" in on_disk['problems'][0]

  def test_a_git_commit_param_not_the_checkouts_is_said(self, checkout, tmp_path):
    doc = tree.start(str(tmp_path), 'tpsim', git_commit=b'0' * 40, managers=[str(checkout)], resolve=no_packages)
    assert any("GitCommit param" in p for p in doc['problems'])


def test_run_writes_it_at_the_start_and_checks_it_after_the_drive(monkeypatch, tmp_path, checkout):
  # run.main with the light-oracle wiring test's fakes: tree.json is written before the bridge runs and checked after it
  from openpilot.tools.sim.bridge.beamng.scenarios import run
  from openpilot.tools.sim.bridge.beamng.scenarios.tests import test_light_oracle_wiring as wiring
  seen = {}

  class Bridge(wiring.FakeBridge):
    def run(self, q):
      seen['at_drive'] = json.load(open(next((tmp_path / 'out').iterdir()) / 'tree.json'))
      return SimpleNamespace(join=lambda: None)
  monkeypatch.setattr(run.tree, 'running_managers', lambda prefix: [str(checkout)])
  monkeypatch.setattr(run.tree, 'package_dirs', own_packages(checkout))
  monkeypatch.setattr(sys, 'argv', ['run.py', 'ped_in_path', '--out', str(tmp_path / 'out')])
  monkeypatch.setattr(run, 'build', lambda *a, **k: SimpleNamespace(spawn=(0., 0., 0.), heading=0., notes={}))
  monkeypatch.setattr(run, 'Params', lambda: SimpleNamespace(get_bool=lambda k: True, get=lambda k: None))
  monkeypatch.setattr(run, 'ensure_token', lambda params: 'tok')
  monkeypatch.setattr(run, 'sweep_game', lambda host, port: None)
  monkeypatch.setattr(run, 'Runner', lambda *a, **k: None)
  monkeypatch.setattr('openpilot.tools.sim.bridge.beamng.bridge.BeamNGBridge', Bridge)
  run.main()
  (run_dir,) = (tmp_path / 'out').iterdir()
  end = json.load(open(run_dir / 'tree.json'))
  assert seen['at_drive']['changed_during_run'] is None and seen['at_drive']['end_checked_at'] is None
  assert end['changed_during_run'] is False and end['clean'] and end['stack_dir'] == os.path.realpath(checkout)


def test_a_refused_expectation_writes_nothing(monkeypatch, tmp_path):
  # an argument error is no run: no run dir, so no tree.json without a result file (a page refuses such a dir as a run
  # killed in its first run)
  from openpilot.tools.sim.bridge.beamng.scenarios import run
  monkeypatch.setattr(run.tree, 'running_managers', lambda prefix: [])
  monkeypatch.setattr(sys, 'argv', ['run.py', 'ped_in_path', '--out', str(tmp_path / 'out'), '--expect', 'handbak:detectorStopped'])
  monkeypatch.setattr(run, 'build', lambda *a, **k: SimpleNamespace(spawn=(0., 0., 0.), heading=0., notes={}))
  monkeypatch.setattr(run, 'Params', lambda: SimpleNamespace(get_bool=lambda k: True, get=lambda k: None))
  monkeypatch.setattr(run, 'ensure_token', lambda params: 'tok')
  monkeypatch.setattr(run, 'sweep_game', lambda host, port: pytest.fail("the game was touched"))
  with pytest.raises(ValueError, match="unknown expectation"):
    run.main()
  assert not (tmp_path / 'out').exists() or not any((tmp_path / 'out').iterdir())
