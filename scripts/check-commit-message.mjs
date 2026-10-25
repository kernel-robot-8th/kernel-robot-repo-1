import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

export const types = ['feat', 'bugfix', 'chore', 'docs', 'test', 'refactor', 'perf', 'style', 'build', 'ci', 'revert'];
const pattern = new RegExp(`^\\[FFT1-[1-9]\\d*\\] (${types.join('|')}): \\S(?:.*\\S)?$`);

export function validMessage(message) {
  const subject = message.split(/\r?\n/, 1)[0];
  return pattern.test(subject) && !/[\u0000-\u001f\u007f]/.test(subject);
}

if (process.argv[1] && fileURLToPath(import.meta.url) === resolve(process.argv[1])) {
  try {
    const file = process.argv[2];
    if (!file || process.argv.length !== 3) {
      throw new Error('Usage: node scripts/check-commit-message.mjs <message-file>');
    }
    // Git-flow integration merges retain Git's generated message.
    const merging = execFileSync('git', ['rev-parse', '--git-path', 'MERGE_HEAD'], { encoding: 'utf8' }).trim();
    let mergeHead = '';
    try {
      mergeHead = readFileSync(merging, 'utf8').trim();
    } catch (error) {
      if (error.code !== 'ENOENT') throw error;
    }
    if (!mergeHead && !validMessage(readFileSync(file, 'utf8'))) {
      throw new Error(`커밋 형식: [FFT1-번호] type: 제목\n예: [FFT1-34] chore: 협업 환경 설정\n허용 type: ${types.join(', ')}`);
    }
  } catch (error) {
    console.error(error.message);
    process.exitCode = 1;
  }
}
