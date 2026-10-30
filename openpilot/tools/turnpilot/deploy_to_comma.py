#!/usr/bin/env python3
"""Install one TurnPilot commit on a comma four over SSH, from this Mac's own clones.

The installer (installer.comma.ai) can't install this fork while opendbc_repo points at the private
kaileh57/turnpilot-opendbc: it clones over unauthenticated https, and no credential may go on the comma. This builds the
tree here instead, with every submodule at its gitlink and every LFS file as its real bytes, copies it to a new directory
on the comma, verifies it, and only then swaps it in. The old /data/openpilot (sunnypilot) is kept as
/data/openpilot.prev-<stamp>; --rollback swaps it back. The comma never fetches anything.

  python3 openpilot/tools/turnpilot/deploy_to_comma.py <comma address> [--commit <sha>] [--dry-run]
  python3 openpilot/tools/turnpilot/deploy_to_comma.py <comma address> --rollback [--prev openpilot.prev-<stamp>]
  python3 openpilot/tools/turnpilot/deploy_to_comma.py --stage-only <dir> [--commit <sha>]

Exit status: 0 done, 1 failed (the output says what state the comma is in), 2 refused before changing anything.
Standard library only, so it runs with the Mac's python3.
"""
import argparse
import datetime
import hashlib
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

BRANCH = "turnpilot"  # the branch name the comma sees: not in RELEASE_BRANCHES, so IsReleaseBranch is 0
ORIGIN_URL = "https://github.com/kaileh57/turnpilot.git"  # credential-free; only read, since DisableUpdates is set
LFS_HEADER = b"version https://git-lfs.github.com/spec/v1\n"
# item by item what the device reads as real bytes: the driving models (big = chestnut), the TurnPilot detectors, the
# chestnut firmware, the AGNOS updater, the UI fonts. Every LFS pointer in the tree is replaced; these are checked by name.
MUST_BE_REAL = ("openpilot/selfdrive/modeld/models/*.pkl", "openpilot/selfdrive/modeld/models/*.onnx",
                "openpilot/selfdrive/turnpilot/models/*", "openpilot/system/hardware/chestnut/*.bin",
                "openpilot/common/hardware/comma/updater", "openpilot/selfdrive/assets/fonts/*")
# free space the comma needs beyond the tree: git-lfs on the comma stores a copy of every LFS file the first time git
# checks the tree (that is done here, before the swap), and the first boot's build writes its outputs
BUILD_HEADROOM = 3 << 30
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=15", "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=4"]
# the comma's git must not read this Mac's config (credential helpers, hooks, lfs smudge that would hit the network)
GIT_ENV = {"GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_NOSYSTEM": "1", "GIT_LFS_SKIP_SMUDGE": "1", "GIT_TERMINAL_PROMPT": "0",
           "GIT_AUTHOR_NAME": "deploy_to_comma", "GIT_AUTHOR_EMAIL": "deploy_to_comma@localhost",
           "GIT_COMMITTER_NAME": "deploy_to_comma", "GIT_COMMITTER_EMAIL": "deploy_to_comma@localhost", "LC_ALL": "C"}

# The comma's half, one POSIX sh script run as `sh -c SCRIPT <name> <action> <args>`. It prints [comma] lines, and exits 3
# with a FAILED line when it stops. Every action but receive/verify/swap/set_param/rollback only reads.
REMOTE_SH = r'''
set -u
act="$1"; shift
say() { printf '  [comma] %s\n' "$*"; }
die() { printf '  [comma] FAILED: %s\n' "$*"; exit 3; }
pval() { cat "$1/params/d/$2" 2>/dev/null || true; }
onroad_state() {
  on=$(pval "$1" IsOnroad); off=$(pval "$1" IsOffroad)
  if [ "$on" = 1 ] || [ "$off" = 0 ]; then echo onroad
  elif [ "$off" = 1 ] || [ "$on" = 0 ]; then echo offroad
  else echo unknown; fi
}
require_offroad() {
  s=$(onroad_state "$1")
  [ "$s" = offroad ] || die "the comma is $s (IsOffroad=$(pval "$1" IsOffroad), IsOnroad=$(pval "$1" IsOnroad)); turn the car off"
}
tree_line() {
  GIT_OPTIONAL_LOCKS=0 git -C "$1" log -1 --format="%h %s" 2>/dev/null | cut -c1-90 || echo "(not a git checkout)"
}
tree_branch() { GIT_OPTIONAL_LOCKS=0 git -C "$1" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "?"; }

case "$act" in
probe)
  root="$1"
  echo "onroad=$(onroad_state "$root")"
  echo "IsOffroad=$(pval "$root" IsOffroad)"
  echo "IsOnroad=$(pval "$root" IsOnroad)"
  echo "DisableUpdates=$(pval "$root" DisableUpdates)"
  [ -d "$root/params/d" ] && echo params=1 || echo params=0
  echo "avail_kb=$(df -Pk "$root" 2>/dev/null | awk 'NR == 2 {print $4}')"
  for t in git tar sha256sum find xargs sort wc mv sync awk readlink cut; do
    command -v "$t" >/dev/null 2>&1 || echo "missing_tool=$t"
  done
  command -v git-lfs >/dev/null 2>&1 && echo git_lfs=1 || echo git_lfs=0
  if [ -d "$root/openpilot" ]; then
    echo "current=$(tree_branch "$root/openpilot") $(tree_line "$root/openpilot")"
    echo "current_origin=$(git -C "$root/openpilot" config --get remote.origin.url 2>/dev/null || true)"
  fi
  for p in "$root"/openpilot.prev-*; do [ -d "$p" ] && echo "prev=${p##*/} $(tree_branch "$p") $(tree_line "$p")"; done
  for p in "$root"/openpilot.tp-incoming-*; do [ -d "$p" ] && echo "leftover=${p##*/} $(du -sk "$p" | cut -f1)"; done
  if [ -f "$root/continue.sh" ]; then
    grep -q "$root/openpilot" "$root/continue.sh" && echo continue_sh=ok || echo continue_sh=other
  else
    echo continue_sh=missing
  fi
  exit 0
  ;;
receive)
  dest="$1"
  [ -e "$dest" ] && die "$dest already exists"
  mkdir "$dest" || die "could not create $dest"
  opts=""
  tar --version 2>/dev/null | grep -q GNU && opts="--warning=no-unknown-keyword"
  # shellcheck disable=SC2086
  tar -C "$dest" $opts -xf - || die "the copy into $dest did not finish"
  sync
  say "received the tree into $dest"
  ;;
verify)
  dest="$1"; sha="$2"; branch="$3"; nfiles="$4"; links="$5"; shift 5
  cd "$dest" || die "no $dest"
  say "checking every file's sha256 against the Mac's manifest"
  sha256sum -c --quiet --strict - || die "files in $dest differ from the Mac's tree"
  n=$(find . -type f | wc -l | tr -d ' ')
  [ "$n" = "$nfiles" ] || die "$dest has $n files, the Mac's tree has $nfiles"
  l=$(find . -type l | LC_ALL=C sort | while IFS= read -r f; do printf '%s -> %s\n' "$f" "$(readlink "$f")"; done | sha256sum | cut -d' ' -f1)
  [ "$l" = "$links" ] || die "the symlinks in $dest differ from the Mac's tree"
  say "$n files and the symlinks match"
  export GIT_OPTIONAL_LOCKS=0
  [ "$(git rev-parse HEAD)" = "$sha" ] || die "HEAD is $(git rev-parse HEAD), not $sha"
  [ "$(git rev-parse --abbrev-ref HEAD)" = "$branch" ] || die "the branch is $(git rev-parse --abbrev-ref HEAD), not $branch"
  [ "$(git rev-parse --abbrev-ref --symbolic-full-name '@{u}')" = "origin/$branch" ] || die "$branch does not track origin/$branch"
  for pair in "$@"; do
    p="${pair%%=*}"; s="${pair#*=}"
    [ "$(git -C "$p" rev-parse HEAD)" = "$s" ] || die "submodule $p is at $(git -C "$p" rev-parse HEAD), not $s"
  done
  st=$(git submodule status) || die "git submodule status failed"
  bad=$(printf '%s\n' "$st" | grep -v '^ ' || true)
  [ -z "$bad" ] || die "submodules not at their gitlinks: $bad"
  say "HEAD $sha on $branch (tracking origin/$branch), $# submodules at their gitlinks"
  unset GIT_OPTIONAL_LOCKS
  say "refreshing git's index (git-lfs re-hashes the model files once; this can take a minute)"
  git update-index -q --refresh >/dev/null 2>&1 || true
  if git diff-index --quiet "origin/$branch" --; then
    say "the tree is clean against origin/$branch"
    echo dirty=0
  elif command -v git-lfs >/dev/null 2>&1; then
    die "the tree differs from $sha: $(git diff-index --name-only "origin/$branch" -- | head -5 | tr '\n' ' ')"
  else
    echo dirty=no-git-lfs
  fi
  ;;
swap)
  root="$1"; incoming="$2"; prev="$3"
  require_offroad "$root"
  [ -d "$incoming" ] || die "no $incoming"
  [ -e "$prev" ] && die "$prev already exists"
  if [ -e "$root/openpilot" ]; then
    mv "$root/openpilot" "$prev" || die "could not rename $root/openpilot; nothing changed"
    say "renamed $root/openpilot to $prev"
  fi
  if mv "$incoming" "$root/openpilot"; then
    sync
    say "moved $incoming to $root/openpilot"
  elif [ -d "$prev" ] && mv "$prev" "$root/openpilot"; then
    die "could not move $incoming into place; put $prev back as $root/openpilot, so nothing changed"
  else
    die "could not move $incoming into place, nor put $prev back: $root/openpilot is MISSING. Run: mv $prev $root/openpilot"
  fi
  ;;
set_param)
  root="$1"; key="$2"; value="$3"
  [ -d "$root/params/d" ] || die "no $root/params/d"
  old=$(cat "$root/params/d/$key" 2>/dev/null || echo unset)
  tmp="$root/params/.tmp_value_deploy_$$"
  printf '%s' "$value" > "$tmp" && mv -f "$tmp" "$root/params/d/$key" && sync || die "could not write $key"
  say "$key: $old -> $value"
  ;;
rollback)
  root="$1"; want="$2"; newprev="$3"
  require_offroad "$root"
  if [ -n "$want" ]; then
    target="$root/$want"
  else
    target=$(ls -d "$root"/openpilot.prev-* 2>/dev/null | LC_ALL=C sort | tail -n 1)
  fi
  [ -n "$target" ] && [ -d "$target" ] || die "no openpilot.prev-* tree in $root to roll back to"
  [ -e "$newprev" ] && die "$newprev already exists"
  say "now:  $root/openpilot is $(tree_branch "$root/openpilot") $(tree_line "$root/openpilot")"
  say "back: ${target##*/} is $(tree_branch "$target") $(tree_line "$target")"
  if [ -e "$root/openpilot" ]; then
    mv "$root/openpilot" "$newprev" || die "could not rename $root/openpilot; nothing changed"
    say "renamed $root/openpilot to $newprev"
  fi
  if mv "$target" "$root/openpilot"; then
    sync
    say "moved $target to $root/openpilot"
  elif mv "$newprev" "$root/openpilot"; then
    die "could not move $target into place; put $newprev back, so nothing changed"
  else
    die "could not move $target into place, nor put $newprev back: $root/openpilot is MISSING. Run: mv $newprev $root/openpilot"
  fi
  ;;
*)
  die "unknown action $act"
  ;;
esac
'''


class Refused(Exception):
  """A precondition failed before anything on the comma changed (exit 2)."""


class Failed(Exception):
  """A step failed (exit 1); the message says what state the comma is in."""


def step(msg: str) -> None:
  print(f"==> {msg}", flush=True)


def info(msg: str) -> None:
  print(f"    {msg}", flush=True)


def gb(n: int) -> str:
  return f"{n / 1e9:.2f} GB"


def git(*args: str, cwd: Path | str | None = None) -> str:
  r = subprocess.run(["git", *args], cwd=cwd, env={**os.environ, **GIT_ENV}, capture_output=True, text=True)
  if r.returncode != 0:
    raise Failed(f"git {' '.join(args)} (in {cwd}) failed: {r.stderr.strip() or r.stdout.strip()}")
  return r.stdout.rstrip("\n")  # not strip(): submodule status's leading space is its "at the gitlink" mark


# git-lfs's clean filter, never its smudge (which would download): the staged index then records each LFS file at its real
# size, as an ordinary LFS checkout does. A pointer-sized index entry reads as modified to git without a content check.
LFS_CLEAN = ["-c", "filter.lfs.process=git-lfs filter-process", "-c", "filter.lfs.clean=git-lfs clean -- %f",
             "-c", "filter.lfs.required=true"]

# ---------------------------------------------------------------------------------------------------------------- Mac


def source_dirs(repo: Path) -> Path:
  """The repo's common git dir: worktrees share its objects, LFS store and submodule clones (modules/<name>)."""
  return Path(git("rev-parse", "--path-format=absolute", "--git-common-dir", cwd=repo))


def resolve_commit(common: Path, commit: str | None) -> str:
  ref = commit or f"refs/heads/{BRANCH}"
  try:
    return git("rev-parse", "--verify", "--quiet", f"{ref}^{{commit}}", cwd=common)
  except Failed:
    raise Failed(f"{ref} is not a commit in {common}") from None


def lfs_pointer(path: Path) -> tuple[str, int] | None:
  if path.is_symlink() or not path.is_file() or path.stat().st_size > 1024:
    return None
  data = path.read_bytes()
  if not data.startswith(LFS_HEADER):
    return None
  fields = dict(line.split(" ", 1) for line in data.decode().splitlines() if " " in line)
  return fields["oid"].removeprefix("sha256:"), int(fields["size"])


def fill_lfs(worktree: Path, store: Path) -> tuple[list[str], int]:
  """Replace every LFS pointer among worktree's tracked files with its object from store (sha256 and size checked)."""
  files = git("ls-files", "-z", cwd=worktree).split("\0")
  replaced, missing, total = [], [], 0
  for rel in filter(None, files):
    ptr = lfs_pointer(worktree / rel)
    if ptr is None:
      continue
    oid, size = ptr
    obj = store / oid[:2] / oid[2:4] / oid
    if not obj.is_file():
      missing.append(rel)
      continue
    mode = (worktree / rel).stat().st_mode
    h = hashlib.sha256()
    tmp = worktree / (rel + ".lfs-tmp")
    with open(obj, "rb") as fi, open(tmp, "wb") as fo:
      while chunk := fi.read(1 << 20):
        h.update(chunk)
        fo.write(chunk)
    if h.hexdigest() != oid or tmp.stat().st_size != size:
      tmp.unlink()
      raise Failed(f"the LFS object for {rel} in {store} is corrupt (sha256 or size differs from the pointer)")
    tmp.chmod(mode & 0o777)
    tmp.replace(worktree / rel)
    replaced.append(rel)
    total += size
  if missing:
    raise Failed(f"{len(missing)} LFS objects are not in {store}: {', '.join(missing[:5])}. " +
                 f"Fetch them first: git -C {store.parent.parent} lfs fetch origin <commit>")
  return replaced, total


def submodules(stage: Path) -> list[tuple[str, str, str]]:
  """(name, path, gitlink) for every gitlink in the staged HEAD."""
  gitlinks = {}
  for line in git("ls-tree", "-r", "HEAD", cwd=stage).splitlines():
    meta, path = line.split("\t", 1)
    mode, _, sha = meta.split()
    if mode == "160000":
      gitlinks[path] = sha
  names = {}
  if (stage / ".gitmodules").exists():
    out = subprocess.run(["git", "config", "-f", ".gitmodules", "--get-regexp", r"^submodule\..*\.path$"], cwd=stage,
                         env={**os.environ, **GIT_ENV}, capture_output=True, text=True).stdout
    for line in out.splitlines():
      key, path = line.split(" ", 1)
      names[path] = key.removeprefix("submodule.").removesuffix(".path")
  unknown = set(gitlinks) - set(names)
  if unknown:
    raise Failed(f"gitlinks with no .gitmodules entry: {sorted(unknown)}")
  return [(names[p], p, s) for p, s in sorted(gitlinks.items())]


def linux_git_config(gitdir: Path) -> None:
  for key in ("core.ignorecase", "core.precomposeunicode"):  # macOS git init sets these true; the comma is Linux
    git("--git-dir", str(gitdir), "config", key, "false")
  if git("--git-dir", str(gitdir), "config", "core.repositoryformatversion") != "0":
    raise Failed(f"{gitdir} is not repository format 0; the comma's git may not read it")
  exts = subprocess.run(["git", "--git-dir", str(gitdir), "config", "--get-regexp", r"^extensions\."],
                        env={**os.environ, **GIT_ENV}, capture_output=True, text=True).stdout.strip()
  if exts:
    raise Failed(f"{gitdir} uses git extensions the comma's git may not read: {exts}")
  (gitdir / "FETCH_HEAD").unlink(missing_ok=True)  # names this Mac's paths


def build_stage(repo: Path, commit: str | None, stage: Path) -> dict:
  """Build the tree the comma gets in stage (which must not exist). Returns its facts."""
  common = source_dirs(repo)
  sha = resolve_commit(common, commit)
  step(f"staging {sha} as branch {BRANCH} in {stage}")
  info(f"source: {common}")
  stage.mkdir(parents=True)
  stage.chmod(0o755)
  any_sha = ["-c", "uploadpack.allowAnySHA1InWant=true", "-c", "protocol.file.allow=always"]
  git("init", "-q", "-b", BRANCH, str(stage))
  git("remote", "add", "origin", ORIGIN_URL, cwd=stage)
  git("config", "remote.origin.fetch", f"+refs/heads/{BRANCH}:refs/remotes/origin/{BRANCH}", cwd=stage)
  git(*any_sha, "fetch", "-q", "--depth", "1", "--no-tags", f"file://{common}", sha, cwd=stage)
  git("checkout", "-q", "-B", BRANCH, sha, cwd=stage)
  git("update-ref", f"refs/remotes/origin/{BRANCH}", sha, cwd=stage)
  git("branch", "-q", f"--set-upstream-to=origin/{BRANCH}", BRANCH, cwd=stage)
  linux_git_config(stage / ".git")

  subs = submodules(stage)
  if subs:
    git("submodule", "init", cwd=stage)  # registers each url, resolved against ORIGIN_URL
  for name, path, link in subs:
    moddir = common / "modules" / name
    url = git("config", f"submodule.{name}.url", cwd=stage)
    have = moddir.is_dir() and subprocess.run(["git", "--git-dir", str(moddir), "cat-file", "-e", f"{link}^{{commit}}"],
                                              env={**os.environ, **GIT_ENV}, capture_output=True).returncode == 0
    src = f"file://{moddir}" if have else url
    info(f"submodule {path} at {link[:12]} from {'this Mac: ' + str(moddir) if have else url}")
    sub = stage / path
    git("init", "-q", str(sub))
    git("remote", "add", "origin", url, cwd=sub)
    git(*any_sha, "fetch", "-q", "--depth", "1", "--no-tags", src, link, cwd=sub)
    git("checkout", "-q", "--detach", link, cwd=sub)
    if (sub / ".gitmodules").exists():
      raise Failed(f"submodule {path} has submodules of its own, which this script doesn't stage")
  if subs:
    git("submodule", "absorbgitdirs", cwd=stage)  # gitdirs into .git/modules/<name>, with relative paths
    for name, _, _ in subs:
      linux_git_config(stage / ".git" / "modules" / name)

  step("putting the real bytes in place of every git-lfs pointer")
  if not shutil.which("git-lfs"):
    raise Failed("git-lfs isn't installed on this Mac (brew install git-lfs)")
  lfs_paths, lfs_bytes = [], 0
  for tree, store, gitdir in [(stage, common / "lfs" / "objects", stage / ".git")] + \
                             [(stage / p, common / "modules" / n / "lfs" / "objects", stage / ".git" / "modules" / n)
                              for n, p, _ in subs]:
    paths, nbytes = fill_lfs(tree, store)
    if paths:
      git(*LFS_CLEAN, "add", "--", *paths, cwd=tree)  # same pointer blobs; the index takes the real files' stat
      shutil.rmtree(gitdir / "lfs", ignore_errors=True)  # git-lfs's copies: the comma's git-lfs makes its own
    lfs_paths += [str((tree / q).relative_to(stage)) for q in paths]
    lfs_bytes += nbytes
  info(f"{len(lfs_paths)} LFS files, {gb(lfs_bytes)}")
  verify_stage(stage, sha, subs)
  facts = manifest(stage)
  facts.update(sha=sha, subs=subs, lfs_bytes=lfs_bytes, lfs_files=len(lfs_paths))
  info(f"{facts['nfiles']} files, {gb(facts['bytes'])}, manifest sha256 {facts['digest'][:16]}")
  return facts


def verify_stage(stage: Path, sha: str, subs: list[tuple[str, str, str]]) -> None:
  step("checking the staged tree")
  checks = [(git("rev-parse", "HEAD", cwd=stage), sha, "HEAD"),
            (git("rev-parse", "--abbrev-ref", "HEAD", cwd=stage), BRANCH, "branch"),
            (git("rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}", cwd=stage), f"origin/{BRANCH}", "upstream")]
  checks += [(git("rev-parse", "HEAD", cwd=stage / p), link, f"submodule {p}") for _, p, link in subs]
  for got, want, what in checks:
    if got != want:
      raise Failed(f"staged {what} is {got}, not {want}")
  for line in git("submodule", "status", cwd=stage).splitlines():
    if not line.startswith(" "):
      raise Failed(f"staged submodule not at its gitlink: {line}")
  for tree in [stage] + [stage / p for _, p, _ in subs]:
    others = git("ls-files", "--others", cwd=tree)
    if others:
      raise Failed(f"untracked files in the staged tree {tree}: {others.splitlines()[:5]}")
  for tree, base in [(stage, sha)] + [(stage / p, link) for _, p, link in subs]:
    for what, args in (("index", ["diff-index", "--cached", "--name-only", base]),
                       ("files", [*LFS_CLEAN, "diff-files", "--name-only", "--ignore-submodules"])):
      diff = git(*args, cwd=tree)
      if diff:
        raise Failed(f"the staged {what} in {tree} differ from {base}: {diff.splitlines()[:5]}")
  pointers = [str(p.relative_to(stage)) for pat in MUST_BE_REAL for p in stage.glob(pat) if lfs_pointer(p)]
  if pointers:
    raise Failed(f"still LFS pointers: {pointers}")
  for f in [stage / ".git" / "config", *[stage / ".git" / "modules" / n / "config" for n, _, _ in subs],
            *[stage / p / ".git" for _, p, _ in subs]]:
    if str(stage) in f.read_text() or "/Users/" in f.read_text():
      raise Failed(f"{f} names a path on this Mac")
  info("HEAD, branch, upstream, submodules, index and files equal to the commits, nothing untracked, no LFS pointers: OK")


def manifest(stage: Path) -> dict:
  """sha256sum -c lines for every regular file, the symlinks' digest as the comma computes it, counts and sizes."""
  files, links, total = [], [], 0
  for dirpath, dirnames, filenames in os.walk(stage):
    for name in dirnames + filenames:
      p = Path(dirpath) / name
      rel = "./" + str(p.relative_to(stage))
      if "\n" in rel or "\\" in rel:
        raise Failed(f"a file name sha256sum can't check: {rel!r}")
      if p.is_symlink():
        links.append(f"{rel} -> {os.readlink(p)}\n")
      elif p.is_file():
        h = hashlib.sha256()
        with open(p, "rb") as f:
          while chunk := f.read(1 << 20):
            h.update(chunk)
        files.append(f"{h.hexdigest()}  {rel}\n")
        total += p.stat().st_size
  links.sort(key=lambda s: s.split(" -> ")[0].encode())
  text = "".join(sorted(files, key=lambda s: s[66:].encode()))
  return {"manifest": text, "digest": hashlib.sha256(text.encode()).hexdigest(), "nfiles": len(files),
          "links": hashlib.sha256("".join(links).encode()).hexdigest(), "bytes": total}


# -------------------------------------------------------------------------------------------------------------- comma


class Comma:
  def __init__(self, address: str, root: str):
    self.host = address if "@" in address else f"comma@{address}"
    self.root = root.rstrip("/") or "/"
    self.ssh = shutil.which("ssh") or "ssh"

  def cmd(self, action: str, *args: str) -> list[str]:
    remote = " ".join(["sh", "-c", shlex.quote(REMOTE_SH), "deploy_to_comma", action, *map(shlex.quote, args)])
    return [self.ssh, *SSH_OPTS, self.host, remote]

  def run(self, action: str, *args: str, stdin: str | None = None, echo: bool = True) -> tuple[int, str]:
    r = subprocess.run(self.cmd(action, *args), input=stdin, capture_output=True, text=True)
    out = r.stdout + r.stderr
    for line in out.splitlines() if echo else []:
      print(line if line.startswith("  [comma]") else f"    {line}", flush=True)
    return r.returncode, out

  def probe(self) -> dict:
    step(f"reading the comma's state ({self.host}, read only)")
    rc, out = self.run("probe", self.root, echo=False)
    if rc != 0:
      raise Refused(f"could not reach the comma over ssh (exit {rc}): {out.strip()[-300:]}")
    state: dict = {"prev": [], "missing_tool": [], "leftover": []}
    for line in out.splitlines():
      if "=" in line and not line.startswith(" "):
        k, v = line.split("=", 1)
        if k in ("prev", "missing_tool", "leftover"):
          state[k].append(v)
        else:
          state[k] = v
    info(f"onroad state: {state.get('onroad')} (IsOffroad={state.get('IsOffroad') or 'unset'}, " +
         f"IsOnroad={state.get('IsOnroad') or 'unset'})")
    info(f"{self.root}/openpilot: {state.get('current', 'absent')}  [{state.get('current_origin', '')}]")
    for p in state["prev"]:
      info(f"kept tree: {p}")
    for p in state["leftover"]:
      name, kb = p.rsplit(" ", 1)
      info(f"leftover partial copy: {self.root}/{name} ({int(kb) / 1e6:.2f} GB); remove it with: " +
           f"ssh {self.host} rm -rf {self.root}/{name}")
    info(f"free space: {gb(int(state.get('avail_kb') or 0) * 1024)}; DisableUpdates={state.get('DisableUpdates') or 'unset'}")
    return state

  def require_offroad(self, state: dict) -> None:
    if state.get("onroad") != "offroad":
      raise Refused(f"the comma is {state.get('onroad')} (IsOffroad={state.get('IsOffroad') or 'unset'}, " +
                    f"IsOnroad={state.get('IsOnroad') or 'unset'}). Turn the car off, wait for the comma to go " +
                    "offroad, and run this again")


def ask_reboot(comma: Comma) -> None:
  try:
    answer = input("Reboot the comma now? [y/N] ").strip().lower()
  except EOFError:
    answer = ""
  if answer not in ("y", "yes"):
    step(f"not rebooting. When ready: ssh {comma.host} sudo reboot")
    return
  step("rebooting the comma")
  r = subprocess.run([comma.ssh, *SSH_OPTS, comma.host, "sudo reboot"], capture_output=True, text=True)
  if r.returncode not in (0, 255):  # 255: the connection dropped as it went down
    print(f"    reboot failed: {(r.stdout + r.stderr).strip()}; reboot it from Settings > Device", flush=True)


def stream_tree(stage: Path, comma: Comma, incoming: str, total: int) -> None:
  """tar the stage on the Mac into the comma's tar, printing progress. Raises Failed on any error."""
  env = {**os.environ, "COPYFILE_DISABLE": "1"}  # no AppleDouble ._ files
  tar = subprocess.Popen(["tar", "--no-xattrs", "--no-acls", "--no-fflags", "--no-mac-metadata", "-C", str(stage),
                          "-cf", "-", "."], stdout=subprocess.PIPE, env=env)
  log = tempfile.TemporaryFile()  # a file, not a pipe: the comma's output can't stall the copy
  ssh = subprocess.Popen(comma.cmd("receive", incoming), stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT)
  assert tar.stdout is not None and ssh.stdin is not None
  sent, last, t0, broken = 0, 0, time.monotonic(), False
  try:
    while chunk := tar.stdout.read(1 << 20):
      try:
        ssh.stdin.write(chunk)
      except BrokenPipeError:
        broken = True
        break
      sent += len(chunk)
      if sent - last >= 64 << 20:
        last = sent
        dt = max(time.monotonic() - t0, 1e-3)
        info(f"sent {gb(sent)} of ~{gb(total)} ({sent / dt / 1e6:.1f} MB/s)")
  finally:
    try:
      ssh.stdin.close()
    except BrokenPipeError:
      broken = True
    tar.stdout.close()
  ssh_rc, tar_rc = ssh.wait(), tar.wait()
  log.seek(0)
  out = log.read().decode(errors="replace")
  log.close()
  for line in out.splitlines():
    print(line if line.startswith("  [comma]") else f"    {line}", flush=True)
  if ssh_rc != 0 or tar_rc not in (0, -13) or broken:
    raise Failed(f"the copy stopped after {gb(sent)} (ssh exit {ssh_rc}, tar exit {tar_rc})")
  info(f"sent {gb(sent)} in {time.monotonic() - t0:.0f} s")


def deploy(args, comma: Comma, stamp: str) -> int:
  state = comma.probe()
  comma.require_offroad(state)
  if state["missing_tool"]:
    raise Refused(f"the comma lacks {', '.join(state['missing_tool'])}")
  if state.get("params") != "1":
    raise Refused(f"no {comma.root}/params/d on the comma, so DisableUpdates can't be set")
  stage_parent = Path(tempfile.mkdtemp(prefix="turnpilot-deploy-"))
  try:
    facts = build_stage(Path(args.repo), args.commit, stage_parent / "openpilot")
    need = facts["bytes"] + facts["lfs_bytes"] + BUILD_HEADROOM
    have = int(state.get("avail_kb") or 0) * 1024
    if have < need:
      raise Refused(f"not enough free space on the comma: need {gb(need)} (the tree {gb(facts['bytes'])}, git-lfs's copy " +
                    f"of the LFS files {gb(facts['lfs_bytes'])}, {gb(BUILD_HEADROOM)} for the first build), have {gb(have)}")
    if state.get("continue_sh") != "ok":
      info(f"WARNING: {comma.root}/continue.sh is {state.get('continue_sh')}: it doesn't name {comma.root}/openpilot, so " +
           "check what it starts after the swap")
    if state.get("git_lfs") != "1":
      info("WARNING: git-lfs isn't on the comma, so git reads the model files as modified and is_dirty is True " +
           "(the files are still checked byte for byte)")
    incoming = f"{comma.root}/openpilot.tp-incoming-{stamp}"
    prev = f"{comma.root}/openpilot.prev-{stamp}"
    current = state.get("current")
    if args.dry_run:
      step("dry run: nothing on the comma changed. A deploy would:")
      info(f"copy {facts['nfiles']} files ({gb(facts['bytes'])}) to {incoming}, and check them")
      info(f"rename {comma.root}/openpilot ({current}) to {prev}" if current else f"({comma.root}/openpilot is absent)")
      info(f"move {incoming} to {comma.root}/openpilot, set DisableUpdates=1, and ask before rebooting")
      return 0

    intact = (f"Both trees are intact: {comma.root}/openpilot is still {current}, and the new tree is at {incoming}. " +
              f"Run this again, or remove it with: ssh {comma.host} rm -rf {incoming}")
    step(f"copying {gb(facts['bytes'])} to {incoming}")
    try:
      stream_tree(stage_parent / "openpilot", comma, incoming, facts["bytes"])
    except Failed as e:
      raise Failed(f"{e}. {intact}") from None

    step(f"verifying {incoming} on the comma")
    pairs = [f"{p}={link}" for _, p, link in facts["subs"]]
    rc, out = comma.run("verify", incoming, facts["sha"], BRANCH, str(facts["nfiles"]), facts["links"], *pairs,
                        stdin=facts["manifest"])
    if rc != 0:
      raise Failed(f"the copy on the comma didn't verify. {intact}")
    info(f"verified: {facts['sha']}, {len(pairs)} submodules, {facts['nfiles']} files, manifest sha256 {facts['digest']}")

    step(f"swapping it in (rename {comma.root}/openpilot to {prev}, then {incoming} to {comma.root}/openpilot)")
    rc, out = comma.run("swap", comma.root, incoming, prev)
    if rc != 0:
      if "MISSING" in out:
        raise Failed(f"the swap failed half way: {comma.root}/openpilot is missing. Run: ssh {comma.host} mv {prev} " +
                     f"{comma.root}/openpilot")
      raise Failed(f"the swap didn't happen. {intact}")

    step("setting DisableUpdates=1, so the updater never fetches from GitHub")
    rc, out = comma.run("set_param", comma.root, "DisableUpdates", "1")
    if rc != 0:
      raise Failed(f"TurnPilot is in {comma.root}/openpilot and the old tree is at {prev}, but DisableUpdates is NOT set. " +
                   f"Don't reboot until it is: ssh {comma.host} \"printf 1 > {comma.root}/params/d/DisableUpdates\"")
    step(f"done: {comma.root}/openpilot is TurnPilot {facts['sha'][:12]}; the old tree ({current}) is {prev}")
    info("the first boot compiles openpilot and flashes the panda; keep the comma powered until the road view shows")
    info(f"to go back: {sys.argv[0]} {comma.host} --rollback")
    ask_reboot(comma)
    return 0
  finally:
    shutil.rmtree(stage_parent, ignore_errors=True)


def rollback(args, comma: Comma, stamp: str) -> int:
  state = comma.probe()
  comma.require_offroad(state)
  names = sorted(p.split(" ", 1)[0] for p in state["prev"])
  if args.prev and args.prev not in names:
    raise Refused(f"no {comma.root}/{args.prev}; kept trees: {names or 'none'}")
  if not names:
    raise Refused(f"no {comma.root}/openpilot.prev-* tree to roll back to")
  target = args.prev or names[-1]
  newprev = f"{comma.root}/openpilot.prev-{stamp}"
  if args.dry_run:
    step(f"dry run: nothing on the comma changed. A rollback would rename {comma.root}/openpilot ({state.get('current')}) " +
         f"to {newprev} and move {target} into its place")
    return 0
  step(f"rolling back: {target} into {comma.root}/openpilot, the current tree to {newprev}")
  rc, out = comma.run("rollback", comma.root, target, newprev)
  if rc != 0:
    if "MISSING" in out:
      raise Failed(f"the rollback failed half way: {comma.root}/openpilot is missing. Run: ssh {comma.host} mv {newprev} " +
                   f"{comma.root}/openpilot")
    raise Failed(f"the rollback didn't happen; {comma.root}/openpilot is as it was")
  step(f"done: {comma.root}/openpilot is {target}'s tree; the one it replaced is {newprev} (--rollback again swaps back)")
  info("DisableUpdates stays 1; turn updates back on in the restored fork's settings if you want them")
  ask_reboot(comma)
  return 0


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("address", nargs="?", help="the comma's address, as comma@<ip> or <ip> (user comma)")
  p.add_argument("--commit", help=f"commit to deploy (default: the local {BRANCH} branch tip)")
  p.add_argument("--dry-run", action="store_true", help="build and check the tree, read the comma's state, change nothing")
  p.add_argument("--rollback", action="store_true", help="swap the newest /data/openpilot.prev-* back into place")
  p.add_argument("--prev", help="with --rollback: the kept tree to restore, e.g. openpilot.prev-20261031-080000")
  p.add_argument("--stage-only", metavar="DIR", help="only build the tree into DIR (must not exist) and keep it; no ssh")
  p.add_argument("--repo", default=str(Path(__file__).resolve().parents[3]), help="source repo (default: this checkout)")
  p.add_argument("--data-root", default="/data", help=argparse.SUPPRESS)  # tests: the fake comma's /data
  p.add_argument("--stamp", default=datetime.datetime.now().strftime("%Y%m%d-%H%M%S"), help=argparse.SUPPRESS)
  args = p.parse_args(argv)
  if args.prev and ("/" in args.prev or not args.prev.startswith("openpilot.prev-")):
    p.error("--prev takes a name like openpilot.prev-20261031-080000")
  try:
    if args.stage_only:
      if Path(args.stage_only).exists():
        raise Failed(f"{args.stage_only} already exists")
      facts = build_stage(Path(args.repo), args.commit, Path(args.stage_only))
      step(f"staged {facts['sha']} in {args.stage_only}: {facts['nfiles']} files, {gb(facts['bytes'])}")
      return 0
    if not args.address:
      p.error("give the comma's address")
    comma = Comma(args.address, args.data_root)
    return rollback(args, comma, args.stamp) if args.rollback else deploy(args, comma, args.stamp)
  except Refused as e:
    print(f"REFUSED: {e}. Nothing on the comma changed.", flush=True)
    return 2
  except Failed as e:
    print(f"FAILED: {e}", flush=True)
    return 1


if __name__ == "__main__":
  sys.exit(main())
