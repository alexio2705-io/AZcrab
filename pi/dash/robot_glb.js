/* Настоящий AZcrab из CAD (vendor/robot.glb, сжат tools/make_robot_glb.mjs)
   вместо мультяшной модели robot3d.js - тот же интерфейс.

   loadRobotGLB(THREE, url) -> Promise<{group, spin(path_mm)}>
   group: начало координат - на полу под центром робота, мм; вперёд - -z,
   вверх - +y, вправо - +x (как у makeRobot3D).
   spin(path_mm) - повернуть колёса на пройденный путь (знак - вперёд/назад).

   В файле модели: метры, вперёд - +x (бампер с концевиками), вверх - +y,
   вправо - +z; начало - посередине между осями. Колёса - узлы WHEEL_<k>_*.
   Нужен THREE.GLTFLoader (vendor/GLTFLoader.js, r147 - последний не модулем). */
(function () {
  'use strict';

  function loadRobotGLB(T, url) {
    return new Promise((resolve, reject) => {
      new T.GLTFLoader().load(url, gltf => resolve(build(T, gltf.scene)), undefined, reject);
    });
  }

  // Цвета - как в CAD. GLTFLoader переводит их в линейные, а пульт рисует без
  // sRGB-выхода (как мультяшную модель) - без обратного перевода всё темнее, а
  // чёрный пластик совсем чёрный. Металл без карты отражений рисуется чёрным -
  // делаем его почти матовым.
  function cadColors(model) {
    const seen = new Set();
    model.traverse(o => {
      if (!o.isMesh || seen.has(o.material)) return;
      seen.add(o.material);
      o.material.color.convertLinearToSRGB();
      if (o.material.metalness > 0.5) o.material.metalness = 0.2;
    });
  }

  function build(T, model) {
    cadColors(model);
    const body = new T.Group();        // система модели, мм, колёса на полу
    body.add(model);
    model.scale.setScalar(1000);       // м -> мм
    body.updateMatrixWorld(true);
    const wheels = wheelPivots(T, model, body);
    body.position.y = -new T.Box3().setFromObject(body).min.y;
    const group = new T.Group();
    group.rotation.y = Math.PI / 2;    // +x модели (перёд) -> -z пульта
    group.add(body);
    // Колесо катится вперёд (+x модели) - поворот вокруг оси +z на -путь/R
    const spin = path => wheels.forEach(w => { w.pivot.rotation.z = -path / w.r; });
    return {group, spin};
  }

  // Каждое колесо - в свою группу с началом на оси колеса, чтобы крутить его
  // на месте. attach() сохраняет положение детали в пространстве.
  function wheelPivots(T, model, body) {
    const parts = {};
    model.traverse(o => {
      const m = /^WHEEL_(\d+)_/.exec(o.name);
      if (m) (parts[m[1]] = parts[m[1]] || []).push(o);
    });
    return Object.values(parts).map(objs => {
      const box = new T.Box3();
      objs.forEach(o => box.expandByObject(o));
      const pivot = new T.Group();
      box.getCenter(pivot.position);
      body.add(pivot);
      pivot.updateMatrixWorld(true);
      objs.forEach(o => pivot.attach(o));
      return {pivot, r: (box.max.y - box.min.y) / 2};
    });
  }

  window.loadRobotGLB = loadRobotGLB;
})();
