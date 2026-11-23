import assert from 'node:assert/strict';
import { execFileSync, spawnSync } from 'node:child_process';
import { copyFileSync, mkdirSync, mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { test } from 'node:test';
import { fileURLToPath } from 'node:url';
import { types, validMessage } from './check-commit-message.mjs';

test('accepts every allowed type, Korean subjects and optional commit bodies', () => {
  for (const type of types) {
    assert.equal(validMessage(`[FFT1-34] ${type}: 작업 내용`), true);
  }
  assert.equal(validMessage('[FFT1-34] feat: 예약 API 추가\r\n\r\n설명\r\n'), true);
  assert.equal(validMessage('[FFT1-1] bugfix: 오류 수정 (#4)'), true);
});

test('real Husky hook rejects invalid commits, works in subdirectories and permits Git merges', () => {
  const root = dirname(dirname(fileURLToPath(import.meta.url)));
  const directory = mkdtempSync(join(tmpdir(), 'roport-hook-test-'));
  const env = { ...process.env, HUSKY: '1' };
  const git = (...args) => execFileSync('git', args, { cwd: directory, env, encoding: 'utf8', stdio: 'pipe' });
  try {
    mkdirSync(join(directory, '.husky'));
    mkdirSync(join(directory, 'scripts'));
    mkdirSync(join(directory, 'nested'));
    for (const file of ['.husky/commit-msg', 'scripts/check-commit-message.mjs']) {
      copyFileSync(join(root, file), join(directory, file));
    }
    git('init', '--initial-branch=main');
    git('config', 'user.name', 'Hook Test');
    git('config', 'user.email', 'hook-test@example.invalid');
    git('config', 'commit.gpgsign', 'false');
    execFileSync(process.execPath, [join(root, 'node_modules/husky/bin.js')], { cwd: directory, env });
    git('commit', '--allow-empty', '-m', '[FFT1-34] test: hook 동작 확인');
    const before = git('rev-parse', 'HEAD');
    const rejected = spawnSync('git', ['commit', '--allow-empty', '-m', 'feat: 티켓 누락'], {
      cwd: join(directory, 'nested'), env, encoding: 'utf8',
    });
    assert.notEqual(rejected.status, 0);
    assert.match(rejected.stderr + rejected.stdout, /FFT1-번호/);
    assert.equal(git('rev-parse', 'HEAD'), before);
    execFileSync('git', ['commit', '--allow-empty', '-m', '[FFT1-34] docs: 하위 디렉토리 확인'], {
      cwd: join(directory, 'nested'), env, stdio: 'pipe',
    });
    git('switch', '-c', 'feature');
    git('commit', '--allow-empty', '-m', '[FFT1-34] test: 기능 브랜치');
    git('switch', 'main');
    git('merge', '--no-ff', 'feature', '-m', 'Merge branch feature');
    assert.equal(git('show', '-s', '--format=%P', 'HEAD').trim().split(' ').length, 2);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test('rejects missing/wrong Jira keys, unsupported types and blank subjects', () => {
  for (const message of [
    '', 'feat: 기능 추가', '[FFT-34] feat: 기능 추가', '[OTHER-34] feat: 기능 추가',
    '[FFT1-] feat: 기능 추가', '[FFT1-0] feat: 기능 추가', '[FFT1-034] feat: 기능 추가',
    '[FFT1-34] fix: 오류 수정', '[FFT1-34] Feat: 기능 추가', '[FFT1-34] feat(scope): 기능 추가',
    '[FFT1-34] feat:', '[FFT1-34] feat:   ', '[FFT1-34] feat: 작업 내용 ',
    ' [FFT1-34] feat: 작업 내용', '[FFT1-34] feat: \t내용', '[FFT1-34] feat: 내용\u0000',
    'Merge branch main', 'fixup! [FFT1-34] feat: 내용',
  ]) {
    assert.equal(validMessage(message), false, JSON.stringify(message));
  }
});

test('CI fetches a missing before commit and still rejects invalid PR titles', () => {
  const root = dirname(dirname(fileURLToPath(import.meta.url)));
  const directory = mkdtempSync(join(tmpdir(), 'roport-ci-range-'));
  const source = join(directory, 'origin');
  const checkout = join(directory, 'checkout');
  const env = { ...process.env, GIT_AUTHOR_NAME: 'CI Test', GIT_AUTHOR_EMAIL: 'ci@example.invalid', GIT_COMMITTER_NAME: 'CI Test', GIT_COMMITTER_EMAIL: 'ci@example.invalid' };
  const git = (...args) => execFileSync('git', args, { cwd: source, env, encoding: 'utf8', stdio: 'pipe' }).trim();
  try {
    mkdirSync(source);
    git('init', '--initial-branch=main');
    git('config', 'commit.gpgsign', 'false');
    mkdirSync(join(source, 'scripts'));
    copyFileSync(join(root, 'scripts/check-commit-message.mjs'), join(source, 'scripts/check-commit-message.mjs'));
    git('add', '.');
    git('commit', '-m', '[FFT1-164] test: base');
    const base = git('rev-parse', 'HEAD');
    git('switch', '-c', 'old');
    git('commit', '--allow-empty', '-m', '[FFT1-164] test: old head');
    const before = git('rev-parse', 'HEAD');
    git('switch', '-c', 'replacement', base);
    git('commit', '--allow-empty', '-m', '[FFT1-164] test: replacement');
    const head = git('rev-parse', 'HEAD');
    execFileSync('git', ['clone', '--no-local', '--single-branch', '--branch', 'replacement', source, checkout], { stdio: 'pipe' });
    assert.notEqual(spawnSync('git', ['cat-file', '-e', `${before}^{commit}`], { cwd: checkout }).status, 0);
    const workflow = readFileSync(join(root, '.github/workflows/commit-message.yml'), 'utf8');
    const match = workflow.match(/        run: \|\n([\s\S]*)$/);
    assert.ok(match, 'commit range step exists');
    const script = match[1].split('\n').map(line => line.replace(/^          /, '')).join('\n');
    const run = (extra = {}) => spawnSync('bash', ['-e', '-c', script], {
      cwd: checkout, env: { ...env, RUNNER_TEMP: directory, EVENT_NAME: 'push', BASE_SHA: before, HEAD_SHA: head, ...extra }, encoding: 'utf8',
    });
    const repaired = run();
    assert.equal(repaired.status, 0, repaired.stderr + repaired.stdout);
    execFileSync('git', ['cat-file', '-e', `${before}^{commit}`], { cwd: checkout, stdio: 'pipe' });
    git('commit', '--allow-empty', '-m', 'invalid commit subject');
    const badHead = git('rev-parse', 'HEAD');
    execFileSync('git', ['fetch', 'origin', badHead], { cwd: checkout, stdio: 'pipe' });
    assert.notEqual(run({ HEAD_SHA: badHead }).status, 0);
    const invalid = run({ EVENT_NAME: 'pull_request', PR_TITLE: '[FFT1-164] fix: invalid' });
    assert.notEqual(invalid.status, 0);
    assert.match(invalid.stderr + invalid.stdout, /FFT1-번호/);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});
