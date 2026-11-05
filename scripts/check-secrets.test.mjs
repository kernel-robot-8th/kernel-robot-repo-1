import assert from 'node:assert/strict';
import { execFileSync, spawnSync } from 'node:child_process';
import { randomBytes } from 'node:crypto';
import { chmodSync, copyFileSync, existsSync, mkdirSync, mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { test } from 'node:test';
import { fileURLToPath } from 'node:url';

const root = dirname(dirname(fileURLToPath(import.meta.url)));

test('Husky rejects staged secrets and push rejects deleted secrets on another branch', () => {
  const directory = mkdtempSync(join(tmpdir(), 'roport-secret-test-'));
  const env = { ...process.env, HUSKY: '1' };
  const run = (command, args, extra = {}) => spawnSync(command, args, { cwd: directory, env, encoding: 'utf8', ...extra });
  const git = (...args) => execFileSync('git', args, { cwd: directory, env, stdio: 'pipe' });
  try {
    for (const name of ['.husky', 'scripts']) mkdirSync(join(directory, name));
    for (const file of ['.husky/pre-commit', '.husky/pre-push', 'scripts/check-secrets.sh', 'package.json']) {
      copyFileSync(join(root, file), join(directory, file));
    }
    git('init', '--initial-branch=main');
    git('config', 'user.name', 'Security Test');
    git('config', 'user.email', 'security-test@example.invalid');
    git('config', 'commit.gpgsign', 'false');
    execFileSync(process.execPath, [join(root, 'node_modules/husky/bin.js')], { cwd: directory, env });
    writeFileSync(join(directory, 'sample.txt'), 'TOKEN=REPLACE_ME\n');
    git('add', 'sample.txt');
    git('commit', '-m', 'safe fixture');
    const remote = join(directory, 'remote.git');
    git('init', '--bare', remote);
    git('remote', 'add', 'origin', remote);
    git('push', 'origin', 'main');

    // Synthetic credentials exist only in this temporary repository, never in a fixture file.
    const token = ['gh', 'p_', randomBytes(18).toString('hex')].join('');
    writeFileSync(join(directory, 'sample.txt'), `TOKEN=${token}\n`);
    git('add', 'sample.txt');
    const commit = run('git', ['commit', '-m', 'must be rejected']);
    assert.notEqual(commit.status, 0);
    const scan = run('sh', ['scripts/check-secrets.sh', '--pre-commit', '--staged', '--verbose']);
    assert.equal(scan.status, 1);
    assert.ok((scan.stdout + scan.stderr).includes('REDACTED'));
    assert.ok(!(scan.stdout + scan.stderr + commit.stdout + commit.stderr).includes(token));

    git('switch', '-c', 'other-branch');
    // Bypass only to construct the hostile historical input that the push guard must reject.
    git('-c', 'core.hooksPath=/dev/null', 'commit', '-m', 'synthetic historical input');
    writeFileSync(join(directory, 'sample.txt'), 'TOKEN=REPLACE_ME\n');
    git('add', 'sample.txt');
    git('commit', '-m', 'remove synthetic input');
    git('switch', 'main');
    const push = run('git', ['push', 'origin', 'other-branch']);
    assert.notEqual(push.status, 0);
    assert.ok(!(push.stdout + push.stderr).includes(token));
    assert.equal(run('git', ['--git-dir', remote, 'show-ref', '--verify', '--quiet', 'refs/heads/other-branch']).status, 1);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test('missing scanner fails closed', () => {
  const result = spawnSync('/bin/sh', [join(root, 'scripts/check-secrets.sh')], {
    env: { PATH: '/nonexistent' }, encoding: 'utf8',
  });
  assert.equal(result.status, 1);
  assert.match(result.stderr, /Gitleaks is required/);
});

test('installer rejects a corrupted download without replacing the existing binary', () => {
  const directory = mkdtempSync(join(tmpdir(), 'roport-install-test-'));
  try {
    const fakeCurl = join(directory, 'curl');
    writeFileSync(fakeCurl, '#!/bin/sh\nfor arg do last=$arg; done\nprintf broken > "$last"\n');
    chmodSync(fakeCurl, 0o755);
    const target = join(directory, 'bin');
    mkdirSync(target);
    writeFileSync(join(target, 'gitleaks'), 'original');
    const result = spawnSync('/bin/sh', [join(root, 'scripts/install-gitleaks.sh'), target], {
      env: { ...process.env, PATH: `${directory}:${process.env.PATH}` }, encoding: 'utf8',
    });
    assert.notEqual(result.status, 0);
    assert.equal(execFileSync('cat', [join(target, 'gitleaks')], { encoding: 'utf8' }), 'original');
    assert.equal(existsSync(join(target, 'gitleaks.tar.gz')), false);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test('ignore local secrets and recordings while retaining public examples and models', () => {
  for (const filename of ['.env.production', '.env.production.example', 'nested/.env.local', 'nested/server.pem', 'id_ed25519', 'credentials.json', 'capture.pcapng', 'session.mcap', 'recordings/camera.mp4']) {
    assert.equal(spawnSync('git', ['check-ignore', '--no-index', '-q', filename], { cwd: root }).status, 0, filename);
  }
  for (const filename of ['.env.example', 'nested/.env.example', 'robot.glb', 'demo.png', 'migration.sql']) {
    assert.equal(spawnSync('git', ['check-ignore', '--no-index', '-q', filename], { cwd: root }).status, 1, filename);
  }
});
