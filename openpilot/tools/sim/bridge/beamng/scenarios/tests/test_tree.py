"""tree.json (scenarios/tree.py): what code a scenario invocation drove on, since initData.dirty is set on every sim run
here. Each test builds a checkout in tmp_path: openpilot/ with a source file, msgq_repo and opendbc_repo as gitlinks with
their own repos, and the documented msgq patch committed and applied, as the main checkout keeps it."""
import json
import os
import subprocess
import sys
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


def no_packages(stack_dir, package):
  return None


def snap(root):
  return tree.snapshot(str(root), resolve=no_packages)


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
    s = tree.snapshot(str(wt), resolve=lambda stack_dir, pkg: found.get(pkg))
    subs = {x['path']: x for x in s['submodules']}
    assert s['clean'] and subs['msgq_repo']['dir'] == os.path.realpath(checkout / 'msgq_repo') and subs['msgq_repo']['patch_ok']
    assert subs['opendbc_repo']['dir'] == os.path.realpath(checkout / 'opendbc_repo')
    # one that resolves nowhere (or to the worktree itself) has no head: not clean
    s = tree.snapshot(str(wt), resolve=lambda stack_dir, pkg: str(wt / 'msgq_repo') if pkg == 'msgq' else found.get(pkg))
    assert not s['clean'] and next(x for x in s['submodules'] if x['path'] == 'msgq_repo')['head'] is None

  def test_the_venvs_resolution_is_the_stacks(self, tmp_path):
    pkg = tmp_path / 'site' / 'zz_tree_probe_pkg'
    write(pkg / '__init__.py', "")
    assert tree.package_dir(str(tmp_path / 'site'), 'zz_tree_probe_pkg') == str(pkg)
    assert tree.package_dir(str(tmp_path), 'zz_tree_probe_missing') is None

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


class TestFindStack:
  def test_the_running_managers_checkout_is_the_stacks(self, checkout, tmp_path):
    cwd = str(checkout / 'openpilot/system/manager')
    assert tree.find_stack('tpsim', managers=[cwd]) == (os.path.realpath(checkout), 'manager', [])
    d, source, problems = tree.find_stack('tpsim', option=str(tmp_path), managers=[cwd])
    assert (d, source) == (os.path.realpath(checkout), 'manager') and 'not the running manager' in problems[0]

  def test_no_manager_or_two_leave_it_unknown_unless_named(self, checkout, tmp_path):
    assert tree.find_stack('tpsim', managers=[])[:2] == (None, 'none')
    assert tree.find_stack('tpsim', option=str(checkout), managers=[])[:2] == (os.path.realpath(checkout), 'option')
    other = tmp_path / 'other'
    repo(other, {tree.MANAGER: ''})
    d, source, problems = tree.find_stack('tpsim', managers=[str(checkout), str(other)])
    assert (d, source) == (None, 'none') and problems[0].startswith('2 openpilot managers')

  def test_managers_found_by_ps_and_lsof(self, monkeypatch):
    monkeypatch.setattr(tree.os.path, 'isdir', lambda p: False if p == '/proc/self' else os.path.isdir(p))
    out = {('ps', '-axww'): "  101 /venv/bin/python3 ./manager.py\n  102 /venv/bin/python3 ./manager.py\n  103 vim manager.txt\n",
           ('ps', '-wwE', '101'): "/venv/bin/python3 ./manager.py PATH=/bin OPENPILOT_PREFIX=tpsim HOME=/h\n",
           ('ps', '-wwE', '102'): "/venv/bin/python3 ./manager.py PATH=/bin HOME=/h\n",
           ('lsof', '101'): "p101\nfcwd\nn/a/tp/openpilot/system/manager\n", ('lsof', '102'): "p102\nfcwd\nn/b\n"}

    def run(cmd, **kw):
      key = (cmd[0], cmd[1]) if cmd[1] == '-axww' else (cmd[0], cmd[1], cmd[3]) if cmd[0] == 'ps' else (cmd[0], cmd[3])
      return SimpleNamespace(stdout=out[key])
    assert tree.manager_dirs('tpsim', run=run) == ['/a/tp/openpilot/system/manager']
    assert tree.manager_dirs('', run=run) == ['/b']


class TestTreeJson:
  def test_start_writes_it_unverified_and_finish_checks_it(self, checkout, tmp_path):
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    cwd = str(checkout / 'openpilot/system/manager')
    head = git(checkout, 'rev-parse', 'HEAD')
    doc = tree.start(str(run_dir), 'tpsim', git_commit=head, managers=[cwd], resolve=no_packages)
    on_disk = json.load(open(run_dir / 'tree.json'))
    assert on_disk['schema'] == 1 and on_disk['clean'] and on_disk['stack_dir'] == os.path.realpath(checkout)
    assert on_disk['changed_during_run'] is None and on_disk['end_checked_at'] is None  # unverified until the end check
    assert on_disk['problems'] == [] and on_disk['stack_git_commit'] == head
    for k in ('head', 'tree', 'openpilot_tree', 'tracked_changes', 'untracked_py', 'submodules', 'written_at'):
      assert k in on_disk
    tree.finish(str(run_dir), doc, resolve=no_packages)
    on_disk = json.load(open(run_dir / 'tree.json'))
    assert on_disk['changed_during_run'] is False and on_disk['end_checked_at'] is not None and 'at_end' not in on_disk

  def test_an_edit_during_the_run_is_changed(self, checkout, tmp_path):
    write(checkout / 'openpilot/selfdrive/x.py', "X = 2\n")  # dirty from the start: the same status line at the end
    doc = tree.start(str(tmp_path), 'tpsim', managers=[str(checkout)], resolve=no_packages)
    write(checkout / 'openpilot/selfdrive/x.py', "X = 3\n")
    tree.finish(str(tmp_path), doc, resolve=no_packages)
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
  monkeypatch.setattr(run.tree, 'manager_dirs', lambda prefix: [str(checkout)])
  monkeypatch.setattr(run.tree, 'package_dir', no_packages)
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
  monkeypatch.setattr(run.tree, 'manager_dirs', lambda prefix: [])
  monkeypatch.setattr(sys, 'argv', ['run.py', 'ped_in_path', '--out', str(tmp_path / 'out'), '--expect', 'handbak:detectorStopped'])
  monkeypatch.setattr(run, 'build', lambda *a, **k: SimpleNamespace(spawn=(0., 0., 0.), heading=0., notes={}))
  monkeypatch.setattr(run, 'Params', lambda: SimpleNamespace(get_bool=lambda k: True, get=lambda k: None))
  monkeypatch.setattr(run, 'ensure_token', lambda params: 'tok')
  monkeypatch.setattr(run, 'sweep_game', lambda host, port: pytest.fail("the game was touched"))
  with pytest.raises(ValueError, match="unknown expectation"):
    run.main()
  assert not (tmp_path / 'out').exists() or not any((tmp_path / 'out').iterdir())
