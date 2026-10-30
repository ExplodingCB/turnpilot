"""deploy_to_comma.py against a fake comma: a temp dir stands in for /data (a sunnypilot git checkout in openpilot/, a
params dir, a route in media/), and an ssh shim on PATH runs the comma's side of every command on this Mac, with no git
config (so no git-lfs filter) unless a test gives the comma one. The source
is a small fixture repo shaped like this fork: LFS pointers at the device's model and font paths with their objects in
.git/lfs, plain-blob TurnPilot detector weights, one submodule whose gitlink is not its checked-out HEAD, and Mac build
outputs lying in both working trees.

TP_DEPLOY_REAL=1 adds one run of the real commit (this checkout's HEAD, ~1 GB) through the same fake comma."""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from openpilot.tools.turnpilot import deploy_to_comma as dtc

SCRIPT = Path(dtc.__file__)
REPO_ROOT = SCRIPT.parents[3]
ISO = {**os.environ, 'GIT_CONFIG_GLOBAL': os.devnull, 'GIT_CONFIG_NOSYSTEM': '1', 'GIT_AUTHOR_NAME': 'fixture',
       'GIT_AUTHOR_EMAIL': 'fixture@localhost', 'GIT_COMMITTER_NAME': 'fixture', 'GIT_COMMITTER_EMAIL': 'fixture@localhost'}

# item 2's paths: the driving models (big = chestnut), the device's fonts; the detector weights are plain blobs here as in
# the fork (openpilot/selfdrive/turnpilot/** -filter)
LFS_FILES = ['openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl',
             'openpilot/selfdrive/modeld/models/big_driving_warp_1344x760_tinygrad.pkl',
             'openpilot/selfdrive/modeld/models/driving_supercombo.onnx',
             'openpilot/selfdrive/modeld/models/dmonitoring_model.onnx',
             'openpilot/selfdrive/assets/fonts/Inter-Regular.ttf']
PLAIN_MODELS = ['openpilot/selfdrive/turnpilot/models/big_detector_1344x760_tinygrad.pkl',
                'openpilot/selfdrive/turnpilot/models/yolo26n.onnx']
REAL_PATHS = LFS_FILES + PLAIN_MODELS

SSH_SHIM = r'''#!/usr/bin/env bash
# fake ssh: "the comma" is this Mac, with /data at the test's temp dir (deploy_to_comma --data-root)
while [ $# -gt 0 ]; do
  case "$1" in
    -o|-p|-i|-l|-F) shift 2 ;;
    -*) shift ;;
    *) break ;;
  esac
done
host="$1"; shift
cmd="$*"
action=$(printf '%s' "$cmd" | awk '{for (i = 1; i <= NF; i++) if ($i == "deploy_to_comma") {print $(i + 1); exit}}')
echo "host=$host action=${action:-$cmd}" >> "$FAKE_COMMA_LOG"
if [ "$cmd" = "sudo reboot" ]; then echo REBOOT >> "$FAKE_COMMA_LOG"; exit 0; fi
export PATH="$FAKE_COMMA_BIN:$PATH"
# the comma's git never reads this Mac's config: none at all by default (git-lfs installed, its filter not set up)
export GIT_CONFIG_GLOBAL="${FAKE_COMMA_GITCONFIG:-/dev/null}" GIT_CONFIG_NOSYSTEM=1
if [ -n "${FAKE_COMMA_CUT_BYTES:-}" ] && [ "$action" = receive ]; then
  head -c "$FAKE_COMMA_CUT_BYTES" | bash -c "$cmd"
  exit 255
fi
bash -c "$cmd"
rc=$?
if [ -n "${FAKE_COMMA_ONROAD_AFTER_RECEIVE:-}" ] && [ "$action" = receive ]; then
  printf 0 > "$FAKE_COMMA_ONROAD_AFTER_RECEIVE"
fi
exit $rc
'''
DF_SHIM = r'''#!/usr/bin/env bash
if [ -n "${FAKE_DF_AVAIL_KB:-}" ]; then
  echo "Filesystem 1024-blocks Used Available Capacity Mounted on"
  echo "/dev/fake 999999999 1 $FAKE_DF_AVAIL_KB 1% /data"
else
  exec /bin/df "$@"
fi
'''
SUDO_SHIM = '#!/usr/bin/env bash\necho "sudo $*" >> "$FAKE_COMMA_LOG"\nexit 1\n'
SHA256SUM_SHIM = '#!/usr/bin/env bash\nexec shasum -a 256 "$@"\n'


def g(cwd, *args, env=ISO):
  return subprocess.run(['git', *args], cwd=cwd, env=env, check=True, capture_output=True, text=True).stdout.rstrip('\n')


def put(root: Path, rel: str, data: bytes, mode=0o644):
  p = root / rel
  p.parent.mkdir(parents=True, exist_ok=True)
  p.write_bytes(data)
  p.chmod(mode)


def lfs_pointer(src: Path, data: bytes) -> bytes:
  oid = hashlib.sha256(data).hexdigest()
  put(src / '.git' / 'lfs' / 'objects' / oid[:2] / oid[2:4], oid, data)
  return f'version https://git-lfs.github.com/spec/v1\noid sha256:{oid}\nsize {len(data)}\n'.encode()


@pytest.fixture(scope='module')
def source(tmp_path_factory):
  """The fixture repo; returns (repo, first commit, second commit = the turnpilot tip, the panda gitlink)."""
  base = tmp_path_factory.mktemp('src')
  sub = base / 'panda_upstream'
  sub.mkdir()
  g(sub, 'init', '-q', '-b', 'master')
  put(sub, 'board/main.c', b'int main(void) { return 0; }\n')
  g(sub, 'add', '-A')
  g(sub, 'commit', '-q', '-m', 'panda 1')
  put(sub, 'board/main.c', b'int main(void) { return 1; }\n')
  g(sub, 'commit', '-q', '-am', 'panda 2')
  panda1, panda2 = g(sub, 'rev-parse', 'HEAD~1'), g(sub, 'rev-parse', 'HEAD')

  src = base / 'turnpilot'
  src.mkdir()
  g(src, 'init', '-q', '-b', 'turnpilot')
  put(src, '.gitattributes', b'*.pkl filter=lfs diff=lfs merge=lfs -text\n*.onnx filter=lfs diff=lfs merge=lfs -text\n' +
                             b'*.ttf filter=lfs diff=lfs merge=lfs -text\nopenpilot/selfdrive/turnpilot/** -filter\n' +
                             b'openpilot/selfdrive/turnpilot/**/*.pkl -text -diff -merge\n')
  put(src, '.gitignore', b'*.o\n/build/\n')
  put(src, 'RELEASES.md', b'Version 0.10.2\n========================\n* TurnPilot\n\n')
  put(src, 'openpilot/common/version.h', b'#define COMMA_VERSION "0.10.2"\n')
  put(src, 'launch_openpilot.sh', b'#!/usr/bin/env bash\nexec ./launch_chffrplus.sh\n', 0o755)
  for i, rel in enumerate(LFS_FILES):
    put(src, rel, lfs_pointer(src, os.urandom(4096 + i)))
  for rel in PLAIN_MODELS:
    put(src, rel, os.urandom(3000))
  (src / 'openpilot' / 'selfdrive' / 'modeld' / 'models' / 'link_to_big.pkl').symlink_to('big_driving_tinygrad.pkl')
  g(src, '-c', 'protocol.file.allow=always', 'submodule', 'add', '-q', str(sub), 'panda')
  g(src / 'panda', 'checkout', '-q', panda1)
  g(src, 'config', '-f', '.gitmodules', 'submodule.panda.url', '../../commaai/panda.git')
  g(src, 'add', '-A')
  g(src, 'commit', '-q', '-m', 'turnpilot 1')
  first = g(src, 'rev-parse', 'HEAD')
  put(src, 'openpilot/selfdrive/turnpilot/turnpilotd.py', b'# second commit\n')
  g(src, 'add', '-A')
  g(src, 'commit', '-q', '-m', 'turnpilot 2')
  second = g(src, 'rev-parse', 'HEAD')
  # what must not go: the submodule's checked-out HEAD is past its gitlink, and Mac build outputs lie in both trees
  g(src / 'panda', 'checkout', '-q', panda2)
  put(src, 'build/selfdrive.o', b'mach-o')
  put(src, 'panda/board/obj/main.o', b'mach-o')
  return src, first, second, panda1


@pytest.fixture
def comma(tmp_path):
  """The fake comma: returns (data root, env for running the script)."""
  root = tmp_path / 'data'
  sp = root / 'openpilot'
  sp.mkdir(parents=True)
  g(sp, 'init', '-q', '-b', 'release-tizi')
  put(sp, 'launch_openpilot.sh', b'#!/usr/bin/env bash\n# sunnypilot\n', 0o755)
  put(sp, 'sunnypilot/models/sp_model.onnx', os.urandom(2000))
  g(sp, 'add', '-A')
  g(sp, 'commit', '-q', '-m', 'sunnypilot')
  g(sp, 'remote', 'add', 'origin', 'https://github.com/sunnyhaibin/sunnypilot.git')
  (sp / '.overlay_init').touch()
  pd = root / 'params' / 'd_abc123'
  pd.mkdir(parents=True)
  (root / 'params' / 'd').symlink_to('d_abc123')
  (pd / 'IsOffroad').write_text('1')
  (pd / 'DisableUpdates').write_text('0')
  put(root, 'media/0/realdata/2026-10-29--10-00-00--0/rlog.zst', os.urandom(5000))
  put(root, 'continue.sh', f'#!/usr/bin/env bash\ncd {root}/openpilot\nexec ./launch_openpilot.sh\n'.encode(), 0o755)

  shims = tmp_path / 'shims'
  comma_bin = tmp_path / 'comma_bin'
  for d, files in ((shims, {'ssh': SSH_SHIM}), (comma_bin, {'df': DF_SHIM, 'sudo': SUDO_SHIM})):
    d.mkdir()
    for name, text in files.items():
      put(d, name, text.encode(), 0o755)
  if not any((Path(p) / 'sha256sum').exists() for p in os.environ['PATH'].split(os.pathsep)):
    put(comma_bin, 'sha256sum', SHA256SUM_SHIM.encode(), 0o755)
  env = {**os.environ, 'PATH': f'{shims}{os.pathsep}{os.environ["PATH"]}', 'FAKE_COMMA_BIN': str(comma_bin),
         'FAKE_COMMA_LOG': str(tmp_path / 'ssh.log')}
  return root, env


def deploy(source_repo, root, env, *args, answer='n', extra_env=None):
  cmd = [sys.executable, str(SCRIPT), 'fakecomma', '--repo', str(source_repo), '--data-root', str(root), *args]
  r = subprocess.run(cmd, env={**env, **(extra_env or {})}, input=answer + '\n', capture_output=True, text=True, timeout=600)
  print(r.stdout, r.stderr)
  return r


def snapshot(root: Path) -> dict:
  out = {}
  for dirpath, dirnames, filenames in os.walk(root):
    for name in dirnames + filenames:
      p = Path(dirpath) / name
      rel = str(p.relative_to(root))
      if p.is_symlink():
        out[rel] = 'link:' + os.readlink(p)
      elif p.is_file():
        out[rel] = hashlib.sha256(p.read_bytes()).hexdigest()
      else:
        out[rel] = 'dir'
  return out


def tree_of(root: Path, name: str) -> dict:
  return {k.split('/', 1)[1]: v for k, v in snapshot(root).items() if k.startswith(name + '/')}


def version_values(tree: Path, gitconfig: str = os.devnull) -> dict:
  """openpilot's own version functions, run the way the device runs them (is_dirty calls get_origin/get_branch with no
  cwd, so cwd is the deployed tree), with the comma's git config (none by default), never this Mac's."""
  code = ('import json; from openpilot.common import git, version\n' +
          't = "."\n' +
          'print(json.dumps({"commit": git.get_commit(t), "short_branch": git.get_short_branch(t), "branch": git.get_branch(t),' +
          ' "origin": git.get_origin(t), "dirty": version.is_dirty(t)}))')
  env = {**os.environ, 'PYTHONPATH': str(REPO_ROOT), 'GIT_CONFIG_GLOBAL': gitconfig, 'GIT_CONFIG_NOSYSTEM': '1'}
  r = subprocess.run([sys.executable, '-P', '-c', code], cwd=tree, env=env, capture_output=True, text=True, check=True)
  return json.loads(r.stdout.strip().splitlines()[-1])


def no_pointers(tree: Path, rels):
  for rel in rels:
    data = (tree / rel).read_bytes()
    assert not data.startswith(dtc.LFS_HEADER), f'{rel} is still an LFS pointer'
    assert len(data) > 1000, rel


def check_deployed(tree: Path, commit: str, gitlink: str, gitconfig: str = os.devnull):
  assert g(tree, 'rev-parse', 'HEAD', env=os.environ) == commit
  assert g(tree / 'panda', 'rev-parse', 'HEAD', env=os.environ) == gitlink
  status = g(tree, 'submodule', 'status', env=os.environ)
  assert status.startswith(' ' + gitlink), status
  assert version_values(tree, gitconfig) == {'commit': commit, 'short_branch': 'turnpilot', 'branch': 'origin/turnpilot',
                                             'origin': dtc.ORIGIN_URL, 'dirty': False}
  assert g(tree / 'panda', 'remote', 'get-url', 'origin', env=os.environ) == 'https://github.com/commaai/panda.git'
  no_pointers(tree, REAL_PATHS)
  assert not (tree / 'build').exists() and not (tree / 'panda' / 'board' / 'obj').exists()
  assert os.readlink(tree / 'openpilot/selfdrive/modeld/models/link_to_big.pkl') == 'big_driving_tinygrad.pkl'
  assert os.access(tree / 'launch_openpilot.sh', os.X_OK)
  for cfg in [tree / '.git' / 'config', tree / '.git' / 'modules' / 'panda' / 'config', tree / 'panda' / '.git']:
    assert '/Users/' not in cfg.read_text() and '/tmp' not in cfg.read_text(), cfg


def prevs(root):
  return sorted(p.name for p in root.glob('openpilot.prev-*'))


def test_fresh_deploy_over_sunnypilot(source, comma):
  src, _, second, panda1 = source
  root, env = comma
  before = snapshot(root)
  sunny = tree_of(root, 'openpilot')
  r = deploy(src, root, env, '--stamp', '20261031-080000')
  assert r.returncode == 0, r.stdout + r.stderr
  check_deployed(root / 'openpilot', second, panda1)
  assert prevs(root) == ['openpilot.prev-20261031-080000']
  assert tree_of(root, 'openpilot.prev-20261031-080000') == sunny
  assert (root / 'params/d/DisableUpdates').read_text() == '1'
  after = snapshot(root)
  assert all(after[k] == v for k, v in before.items() if k.startswith(('media/', 'continue.sh')))
  assert not list(root.glob('openpilot.tp-incoming-*'))
  assert 'REBOOT' not in Path(env['FAKE_COMMA_LOG']).read_text()
  assert 'host=comma@fakecomma' in Path(env['FAKE_COMMA_LOG']).read_text()
  assert 'Reboot' in r.stdout


def test_deploy_over_earlier_turnpilot_deploy(source, comma):
  src, first, second, panda1 = source
  root, env = comma
  assert deploy(src, root, env, '--commit', first, '--stamp', '20261031-080000').returncode == 0
  check_deployed(root / 'openpilot', first, panda1)
  r = deploy(src, root, env, '--stamp', '20261031-090000', answer='y')
  assert r.returncode == 0, r.stdout
  check_deployed(root / 'openpilot', second, panda1)
  assert prevs(root) == ['openpilot.prev-20261031-080000', 'openpilot.prev-20261031-090000']
  assert g(root / 'openpilot.prev-20261031-090000', 'rev-parse', 'HEAD', env=os.environ) == first
  assert g(root / 'openpilot.prev-20261031-080000', 'log', '-1', '--format=%s', env=os.environ) == 'sunnypilot'
  assert 'REBOOT' in Path(env['FAKE_COMMA_LOG']).read_text()


# what `git lfs install` writes to a user's global config
LFS_INSTALLED = ('[filter "lfs"]\n\tclean = git-lfs clean -- %f\n\tsmudge = git-lfs smudge -- %f\n' +
                 '\tprocess = git-lfs filter-process\n\trequired = true\n')


@pytest.mark.parametrize('comma_gitconfig', ['', LFS_INSTALLED], ids=['no-lfs-filter', 'lfs-installed'])
def test_clean_whatever_the_commas_git_config(source, comma, tmp_path, comma_gitconfig):
  """Whether AGNOS sets up git-lfs's filter for user comma is unverified. With no filter in its config, or with git lfs
  install's, the comma's check passes and the device's own is_dirty() reads False, also once git has to hash every LFS
  file again (their stat changed), as after a copy or a touch."""
  src, _, second, panda1 = source
  root, env = comma
  cfg = tmp_path / 'comma_gitconfig'
  cfg.write_text(comma_gitconfig)
  r = deploy(src, root, env, extra_env={'FAKE_COMMA_GITCONFIG': str(cfg)})
  assert r.returncode == 0, r.stdout
  tree = root / 'openpilot'
  check_deployed(tree, second, panda1, str(cfg))
  for t, config in ((1e9, os.devnull), (1.1e9, str(cfg))):
    for rel in LFS_FILES:
      os.utime(tree / rel, (t, t))  # whole seconds away from the index's stat, so the next refresh hashes them again
    assert version_values(tree, config)['dirty'] is False, config


@pytest.mark.parametrize('param,value', [('IsOffroad', '0'), ('IsOnroad', '1')])
def test_refuses_onroad(source, comma, param, value):
  src, *_ = source
  root, env = comma
  (root / 'params/d' / param).write_text(value)
  before = snapshot(root)
  r = deploy(src, root, env)
  assert r.returncode == 2
  assert 'onroad' in r.stdout
  assert snapshot(root) == before


def test_refuses_when_it_goes_onroad_during_the_copy(source, comma):
  src, *_ = source
  root, env = comma
  sunny = tree_of(root, 'openpilot')
  r = deploy(src, root, env, extra_env={'FAKE_COMMA_ONROAD_AFTER_RECEIVE': str(root / 'params/d/IsOffroad')})
  assert r.returncode != 0
  assert 'onroad' in r.stdout and 'Both trees are intact' in r.stdout
  assert tree_of(root, 'openpilot') == sunny
  assert not prevs(root)
  assert len(list(root.glob('openpilot.tp-incoming-*'))) == 1
  assert (root / 'params/d/DisableUpdates').read_text() == '0'


def test_refuses_short_disk(source, comma):
  src, *_ = source
  root, env = comma
  before = snapshot(root)
  r = deploy(src, root, env, extra_env={'FAKE_DF_AVAIL_KB': '100000'})
  assert r.returncode == 2
  assert 'free space' in r.stdout
  assert snapshot(root) == before


def test_copy_that_fails_midway_leaves_both_trees(source, comma):
  src, *_ = source
  root, env = comma
  sunny = tree_of(root, 'openpilot')
  r = deploy(src, root, env, extra_env={'FAKE_COMMA_CUT_BYTES': '20000'})
  assert r.returncode == 1
  assert tree_of(root, 'openpilot') == sunny
  assert not prevs(root)
  incoming = list(root.glob('openpilot.tp-incoming-*'))
  assert len(incoming) == 1 and incoming[0].name in r.stdout
  assert 'Both trees are intact' in r.stdout
  assert (root / 'params/d/DisableUpdates').read_text() == '0'


def test_rollback(source, comma):
  src, _, second, panda1 = source
  root, env = comma
  sunny = tree_of(root, 'openpilot')
  assert deploy(src, root, env, '--stamp', '20261031-080000').returncode == 0
  turnpilot = tree_of(root, 'openpilot')

  (root / 'params/d/IsOffroad').write_text('0')
  r = deploy(src, root, env, '--rollback', '--stamp', '20261031-100000')
  assert r.returncode == 2 and 'onroad' in r.stdout
  assert tree_of(root, 'openpilot') == turnpilot

  (root / 'params/d/IsOffroad').write_text('1')
  r = deploy(src, root, env, '--rollback', '--stamp', '20261031-100000')
  assert r.returncode == 0, r.stdout
  assert tree_of(root, 'openpilot') == sunny
  assert prevs(root) == ['openpilot.prev-20261031-100000']
  check_deployed(root / 'openpilot.prev-20261031-100000', second, panda1)
  assert (root / 'media/0/realdata/2026-10-29--10-00-00--0/rlog.zst').exists()

  # a second rollback swaps TurnPilot back
  r = deploy(src, root, env, '--rollback', '--stamp', '20261031-110000')
  assert r.returncode == 0, r.stdout
  check_deployed(root / 'openpilot', second, panda1)
  assert tree_of(root, 'openpilot.prev-20261031-110000') == sunny


def test_dry_run_touches_nothing(source, comma):
  src, *_ = source
  root, env = comma
  before = snapshot(root)
  r = deploy(src, root, env, '--dry-run')
  assert r.returncode == 0, r.stdout
  assert 'dry run' in r.stdout.lower()
  assert snapshot(root) == before
  assert 'REBOOT' not in Path(env['FAKE_COMMA_LOG']).read_text()
  assert {line.split('action=')[1] for line in Path(env['FAKE_COMMA_LOG']).read_text().splitlines()} == {'probe'}

  r = deploy(src, root, env, '--dry-run', '--rollback')
  assert r.returncode == 2 and 'no ' in r.stdout  # nothing kept to roll back to yet
  assert snapshot(root) == before


@pytest.mark.skipif(not os.environ.get('TP_DEPLOY_REAL'), reason='set TP_DEPLOY_REAL=1 to stage and deploy the real commit')
def test_real_commit(comma):
  root, env = comma
  head = g(REPO_ROOT, 'rev-parse', 'HEAD', env=os.environ)
  r = deploy(REPO_ROOT, root, env, '--commit', head, '--stamp', '20261031-080000')
  assert r.returncode == 0, r.stdout
  tree = root / 'openpilot'
  assert g(tree, 'rev-parse', 'HEAD', env=os.environ) == head
  for line in g(REPO_ROOT, 'ls-tree', head, env=os.environ).splitlines():
    mode, _, sha, path = line.split(None, 3)
    if mode == '160000':
      assert g(tree / path, 'rev-parse', 'HEAD', env=os.environ) == sha, path
  v = version_values(tree)
  assert v == {'commit': head, 'short_branch': 'turnpilot', 'branch': 'origin/turnpilot', 'origin': dtc.ORIGIN_URL,
               'dirty': False}
  real = [p for pat in dtc.MUST_BE_REAL for p in sorted(str(q.relative_to(tree)) for q in tree.glob(pat))]
  assert 'openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl' in real
  assert 'openpilot/selfdrive/turnpilot/models/big_detector_1344x760_tinygrad.pkl' in real
  for rel in real:
    assert not (tree / rel).read_bytes()[:len(dtc.LFS_HEADER)] == dtc.LFS_HEADER, rel
  code = ('import json; from openpilot.common.version import get_build_metadata as b; m = b(".");' +
          'print(json.dumps([m.channel, m.openpilot.version, m.openpilot.git_commit, m.openpilot.is_dirty]))')
  out = subprocess.run([sys.executable, '-c', code], cwd=tree, capture_output=True, text=True, check=True,
                       env={**os.environ, 'PYTHONPATH': str(REPO_ROOT), 'GIT_CONFIG_GLOBAL': os.devnull,
                            'GIT_CONFIG_NOSYSTEM': '1'}).stdout.strip().splitlines()[-1]
  channel, _, commit, dirty = json.loads(out)
  assert (channel, commit, dirty) == ('turnpilot', head, False)
