import assert from 'node:assert/strict';
import { execFileSync, spawnSync } from 'node:child_process';
import { copyFileSync, mkdirSync, mkdtempSync, rmSync } from 'node:fs';
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
