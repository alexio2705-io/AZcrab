// CAD-модель робота -> pi/dash/vendor/robot.glb для пульта (robot_glb.js).
//
// Исходник (экспорт из Fusion через three.js, ~26 МБ, 750 тыс. треугольников)
// для телефона и для Pi, которая отдаёт файл во время заезда, тяжёл. Здесь:
//   1. колёса wheel_v2_v10N -> узлы WHEEL_<k>_<i> (их пульт крутит по
//      одометрии), у всех остальных узлов и сеток имя стирается - иначе
//      gltfpack -kn не склеит ни одной детали;
//   2. gltfpack: 10% геометрии (-si 0.1 -sa), квантование (KHR_mesh_quantization -
//      его GLTFLoader понимает сам, без декодера), колёса отдельно (-kn).
//
// Запуск (нужен Node; пакеты ставятся во временную папку, не в репозиторий):
//   cd $(mktemp -d) && npm i @gltf-transform/core@4 gltfpack@0.22
//   node <репо>/pi/dash/tools/make_robot_glb.mjs robot_v3_final.glb <репо>/pi/dash/vendor/robot.glb
import {execFileSync} from 'node:child_process';
import {NodeIO} from '@gltf-transform/core';

const [src, dst, ratio = '0.1'] = process.argv.slice(2);
if (!src || !dst) {
  console.error('usage: node make_robot_glb.mjs <in.glb> <out.glb> [доля геометрии, 0.1]');
  process.exit(1);
}
const io = new NodeIO();
const doc = await io.read(src);
const wheels = doc.getRoot().listNodes().filter(n => /^wheel_v2_v10\d$/.test(n.getName()));
if (wheels.length !== 4) throw new Error(`колёс wheel_v2_v10N: ${wheels.length}, а надо 4`);
const wheelOf = new Map();
wheels.forEach((w, k) => w.traverse(n => wheelOf.set(n, k)));
let i = 0;
for (const n of doc.getRoot().listNodes()) {
  n.setName(wheelOf.has(n) && n.getMesh() ? `WHEEL_${wheelOf.get(n)}_${i++}` : '');
  n.getMesh()?.setName('');
}
const marked = dst + '.marked.glb';
await io.write(marked, doc);
execFileSync('npx', ['gltfpack', '-i', marked, '-o', dst, '-si', ratio, '-sa', '-kn'], {stdio: 'inherit'});
(await import('node:fs')).unlinkSync(marked);
console.log(`${dst}: колёс 4, сеток колёс ${i}`);
