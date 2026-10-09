/* AZcrab в 3D, мультяшно: toon-заливка в три тона и чёрный контур.
   Один файл на пульт (pi/dash) и просмотрщик симулятора (sim/viewer/vendor -
   копию туда кладёт sim/viewer/build.py).

   makeRobot3D(THREE) -> {group, spin(path_mm)}
   group: начало координат - на полу под центром робота, мм; вперёд - -z,
   вверх - +y, вправо - +x (как в обоих местах, где модель стоит).
   spin(path_mm) - повернуть колёса на пройденный путь (знак - вперёд/назад).

   По фото робота: четыре больших колеса с кремовыми рифлёными шинами и
   чёрными дисками на 4 отверстия, низкий чёрный корпус между колёсами,
   белый бампер с концевиками и красной платой, сверху Raspberry Pi 5
   портами вперёд, над ней на мостике лидар LD19, по бокам - камеры. Робот симметричный; перёд выдаёт только красная плата
   у бампера и оранжевая метка на переднем бампере. */
(function () {
  'use strict';

  const DIM = {
    wheelR: 44, tireR: 10, wheelW: 22,   // колесо: радиус по шине, толщина шины, ширина
    axle: 47,                            // оси от центра (вперёд/назад)
    track: 63,                           // середина колеса от оси симметрии
    bodyW: 96, bodyL: 150,
    nose: 96.5,                          // бампер от центра (как ROBOT_LEN_MM / 2)
    lidarZ: 130,                         // плоскость скана
  };

  function makeRobot3D(T) {
    const OUT = 1.8;                     // толщина контура, мм
    const grad = (() => {               // три тона: тень, полутон, свет
      const d = new Uint8Array([90, 90, 90, 255, 175, 175, 175, 255, 255, 255, 255, 255]);
      const t = new T.DataTexture(d, 3, 1, T.RGBAFormat);
      t.minFilter = t.magFilter = T.NearestFilter;
      t.needsUpdate = true;
      return t;
    })();
    const mats = {};
    const toonMat = c => mats[c] || (mats[c] = new T.MeshToonMaterial({color: c, gradientMap: grad}));
    const lineMat = new T.MeshBasicMaterial({color: 0x0b0b0b, side: T.BackSide});

    // деталь + контур: копия той же геометрии изнутри наружу, раздутая на OUT
    // по каждой оси отдельно - у коробок контур ровный, у цилиндров почти
    function part(geo, color, outline = true) {
      const g = new T.Group();
      g.add(new T.Mesh(geo, toonMat(color)));
      if (outline) {
        geo.computeBoundingBox();
        const s = new T.Vector3(); geo.boundingBox.getSize(s);
        const c = new T.Vector3(); geo.boundingBox.getCenter(c);
        const o = new T.Mesh(geo, lineMat);
        o.scale.set((s.x + 2 * OUT) / Math.max(s.x, 1e-3), (s.y + 2 * OUT) / Math.max(s.y, 1e-3), (s.z + 2 * OUT) / Math.max(s.z, 1e-3));
        o.position.copy(c).multiplyScalar(1 - o.scale.x).setY(c.y * (1 - o.scale.y)).setZ(c.z * (1 - o.scale.z));
        g.add(o);
      }
      return g;
    }
    const at = (obj, x, y, z) => { obj.position.set(x, y, z); return obj; };
    const rbox = (w, h, d, r, color, outline) => {
      // скруглённая коробка: экструзия скруглённого прямоугольника (вид сбоку)
      const s = new T.Shape(), x = -d / 2, y = -h / 2;
      r = Math.min(r, h / 2, d / 2);
      s.moveTo(x + r, y); s.lineTo(x + d - r, y); s.quadraticCurveTo(x + d, y, x + d, y + r);
      s.lineTo(x + d, y + h - r); s.quadraticCurveTo(x + d, y + h, x + d - r, y + h);
      s.lineTo(x + r, y + h); s.quadraticCurveTo(x, y + h, x, y + h - r);
      s.lineTo(x, y + r); s.quadraticCurveTo(x, y, x + r, y);
      const g = new T.ExtrudeGeometry(s, {depth: w, bevelEnabled: false, curveSegments: 6});
      g.translate(0, 0, -w / 2); g.rotateY(Math.PI / 2);
      return part(g, color, outline);
    };
    const box = (w, h, d, color, outline) => part(new T.BoxGeometry(w, h, d), color, outline);
    const cyl = (r, h, color, seg = 28, outline) => part(new T.CylinderGeometry(r, r, h, seg), color, outline);
    const tube = (pts, r, color) => part(new T.TubeGeometry(new T.CatmullRomCurve3(pts.map(p => new T.Vector3(...p))), 48, r, 8, false), color, false);

    const C = {
      tire: 0xf1e9d0, tread: 0xe8dfc2, hub: 0x1c1c1c, hole: 0x3a3a3a,
      body: 0x232323, deck: 0x151515, white: 0xf4f4f0, red: 0xd8343a,
      pi: 0x2f8f4e, port: 0xb9bec4, lidar: 0x1e1e1e, sticker: 0x3a78c9,
      cam: 0x1a1a1a, lens: 0x2b5fae, yellow: 0xf2c230, acc: 0xf2a900,
    };
    const D = DIM;
    const robot = new T.Group();
    robot.name = 'AZcrab';

    // ── колёса ──────────────────────────────────────────
    const wheels = [];
    function wheel(side) {
      const w = new T.Group();
      const tire = new T.TorusGeometry(D.wheelR - D.tireR, D.tireR, 14, 48);
      tire.rotateY(Math.PI / 2); tire.scale(D.wheelW / (2 * D.tireR), 1, 1);
      w.add(part(tire, C.tire));
      // рифление шины - поперечные гребни по окружности
      const rib = new T.BoxGeometry(D.wheelW * 0.86, 2.0, 2.6);
      for (let i = 0; i < 48; i++) {
        const a = i / 48 * Math.PI * 2;
        const m = new T.Mesh(rib, toonMat(C.tread));
        m.position.set(0, Math.cos(a) * (D.wheelR - 0.2), Math.sin(a) * (D.wheelR - 0.2));
        m.rotation.x = a;
        w.add(m);
      }
      // диск: чёрный, 4 отверстия по кругу + центр, снаружи
      const hubG = new T.CylinderGeometry(D.wheelR - D.tireR * 1.7, D.wheelR - D.tireR * 1.7, D.wheelW * 0.8, 36);
      hubG.rotateZ(Math.PI / 2);
      w.add(part(hubG, C.hub));
      const holeG = new T.CylinderGeometry(3.3, 3.3, 1.5, 14); holeG.rotateZ(Math.PI / 2);
      const face = side * D.wheelW * 0.4 + side * 0.4;
      for (let i = 0; i < 4; i++) {
        const a = i / 4 * Math.PI * 2 + Math.PI / 4;
        w.add(at(new T.Mesh(holeG, toonMat(C.hole)), face, Math.cos(a) * 11, Math.sin(a) * 11));
      }
      w.add(at(new T.Mesh(holeG, toonMat(C.hole)), face, 0, 0));
      return w;
    }
    for (const sz of [-1, 1]) for (const sx of [-1, 1]) {
      const w = wheel(sx);
      w.position.set(sx * D.track, D.wheelR, sz * D.axle);
      robot.add(w); wheels.push(w);
    }

    // ── корпус ──────────────────────────────────────────
    robot.add(at(rbox(D.bodyW, 30, D.bodyL, 10, C.body), 0, 34, 0));            // нижний корпус
    robot.add(at(rbox(D.bodyW + 8, 5, D.bodyL - 8, 3, C.deck), 0, 60, 0));       // плата-основание

    // бамперы: белая П-образная рамка спереди и сзади (робот симметричный)
    for (const sz of [-1, 1]) {
      robot.add(at(rbox(D.bodyW + 22, 10, 9, 4, C.white), 0, 24, sz * (D.nose - 4.5)));
      for (const sx of [-1, 1]) {
        robot.add(at(box(8, 9, 22, C.white), sx * (D.bodyW / 2 - 6), 24, sz * (D.nose - 19)));
        robot.add(at(box(9, 8, 7, C.deck), sx * 22, 31, sz * (D.nose - 14)));   // концевики
      }
    }
    robot.add(at(box(34, 8, 16, C.red), 0, 38, -(D.nose - 26)));               // красная плата - перёд
    robot.add(at(box(30, 4, 10.5, C.acc, false), 0, 27.5, -(D.nose - 4.5)));    // оранжевая метка на переднем бампере

    // жёлтые провода от концевиков
    for (const sx of [-1, 1]) {
      robot.add(tube([[sx * 22, 34, -(D.nose - 14)], [sx * 30, 52, -(D.nose - 22)], [sx * 40, 48, -54], [sx * 44, 58, -40]], 1.4, C.yellow));
    }

    // ── верхний ярус ────────────────────────────────────
    // Raspberry Pi 5: плата 85 x 56, торец с портами (2 сдвоенных USB +
    // Ethernet) смотрит вперёд. Локально: x - короткая сторона, z - длинная,
    // порты на -z.
    const pi = new T.Group();
    pi.add(rbox(56, 1.6, 85, 3, C.pi));
    for (const x of [-19, -1]) pi.add(at(box(13, 15.6, 17.5, C.port), x, 8.6, -42.5 + 6.5));   // USB
    pi.add(at(box(16, 13.5, 21, C.port), 17.8, 7.6, -42.5 + 8.5));                          // Ethernet
    pi.add(at(box(5, 8.5, 51, 0x111111), -24.5, 5, 6));       // GPIO 2x20 вдоль длинной стороны
    pi.add(at(box(15, 2.2, 15, 0xa6acb2), 4, 1.9, 14));       // BCM2712 под крышкой
    pi.add(at(box(8, 1.6, 8, 0x2a2a2a), 8, 1.6, -8));         // RP1
    for (const z of [26, 34]) pi.add(at(box(3, 3, 7, 0x8a9096), 27, 2.3, z));   // micro HDMI
    robot.add(at(pi, 0, 63.6, -20));

    // Стойка лидара - мостик над Pi на четырёх ногах, сверху LD19:
    // квадратное основание ~38.6 x 38.9 со скруглёнными углами и наклейкой,
    // на нём вращающаяся головка; скан - на LIDAR_Z.
    const LT = 104;                                          // верх стойки
    robot.add(at(rbox(62, 5, 56, 6, C.lidar), 0, LT - 2.5, 8));
    for (const sx of [-1, 1]) for (const sz of [-1, 1])
      robot.add(at(box(6, LT - 62, 6, C.lidar), sx * 31, (LT + 62) / 2 - 2.5, 8 + sz * 24));
    robot.add(at(rbox(38.9, 15, 38.6, 6, C.lidar), 0, LT + 7.5, 8));
    robot.add(at(box(16, 6, 0.8, C.sticker, false), 0, LT + 7.5, 8 - 19.7));   // наклейка LDROBOT
    robot.add(at(cyl(17, 18.5, 0x262626, 36), 0, LT + 15 + 9.25, 8));
    robot.add(at(cyl(17.4, 6, 0x3b3b3b, 36, false), 0, D.lidarZ, 8));         // окно дальномера
    robot.add(at(cyl(15.5, 1.5, 0x313131, 36, false), 0, LT + 34, 8));        // крышка головки

    // камеры по бокам: чёрная плата + синий объектив наружу
    for (const sx of [-1, 1]) {
      const cam = new T.Group();
      cam.add(rbox(4, 30, 30, 4, C.cam));
      const lens = new T.CylinderGeometry(6, 7, 7, 18); lens.rotateZ(Math.PI / 2);
      cam.add(at(part(lens, 0x0f0f0f), sx * 5, -2, 0));
      const glass = at(new T.Mesh(new T.CircleGeometry(4.2, 18), new T.MeshBasicMaterial({color: C.lens})), sx * 8.6, -2, 0);
      glass.rotation.y = sx * Math.PI / 2; cam.add(glass);
      for (const dz of [-9, 9]) {
        const led = at(new T.Mesh(new T.CircleGeometry(1.6, 10), new T.MeshBasicMaterial({color: C.red})), sx * 2.2, 10, dz);
        led.rotation.y = sx * Math.PI / 2; cam.add(led);
      }
      robot.add(at(cam, sx * (D.bodyW / 2 - 4), 84, 8));
    }
    // коробка аккумулятора/платы сзади
    robot.add(at(rbox(40, 20, 26, 5, C.deck), 26, 73, 52));

    function spin(path_mm) {
      const a = -path_mm / D.wheelR;
      for (const w of wheels) w.rotation.x = a;
    }
    return {group: robot, spin, dim: D};
  }

  window.makeRobot3D = makeRobot3D;
})();
