const $ = (id) => document.getElementById(id);
const host = $('view');

function showError(error) {
  $('error').textContent = `读取失败：${error.message}`;
  $('error').hidden = false;
  $('loading').hidden = true;
}

async function main() {
  const query = new URLSearchParams(location.search);
  const configReference = query.get('data');

  if (!configReference && query.has('service')) {
    const target = new URL(query.get('service'), location.href);
    if (!['http:', 'https:'].includes(target.protocol)) {
      throw new Error('地图服务地址需要使用 http 或 https。');
    }
    target.pathname = '/';
    target.hash = '';
    target.searchParams.set('tab', 'map');
    if (query.get('map')) target.searchParams.set('map', query.get('map'));
    location.replace(target.href);
    return;
  }

  if (!configReference) {
    throw new Error(
        '缺少 data 参数。离线查看使用 ?data=数据说明.json；正式地图使用 ?service=操作台地址&map=地图名。',
    );
  }

  const THREE = await import('https://esm.sh/three@0.170.0');

  const configUrl = new URL(configReference, location.href);

  async function read(url, format) {
    const response = await fetch(url);
    if (!response.ok) {
      throw new Error(`${url.pathname} · ${response.status}`);
    }
    return response[format]();
  }

  const dataUrl = (path) => new URL(path, configUrl);
  const config = await read(configUrl, 'json');

  document.title = config.documentTitle;
  host.setAttribute('aria-label', config.ariaLabel);
  $('map-title').textContent = config.title;
  $('map-subtitle').textContent = config.subtitle;
  $('details-title').textContent = `${config.title} ${config.subtitle}`;
  $('details-subtitle').textContent = config.summary;
  $('point-count').textContent = `${config.stats.points.toLocaleString()} 点`;
  $('occupied-count').textContent =
      `${config.stats.occupied.toLocaleString()} 个`;
  $('resolution').textContent = config.stats.resolution;
  $('frame-count').textContent = `${config.stats.frames.toLocaleString()} 帧`;
  $('map-name').textContent = config.mapName;
  $('source-note').textContent = config.source.description;
  $('limits-note').textContent = config.source.limits;
  $('focus-label').textContent = config.presets.focus.label;
  $('focus').title = config.presets.focus.label;

  const ceiling = $('ceiling');
  ceiling.min = config.ceiling.min;
  ceiling.max = config.ceiling.max;
  ceiling.step = config.ceiling.step;
  ceiling.value = config.ceiling.value;

  const renderer = new THREE.WebGLRenderer({antialias: true});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  renderer.localClippingEnabled = true;
  renderer.setClearColor('#121617');
  host.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(43, 1, 0.02, 700);
  camera.up.set(0, 0, 1);
  const plane = new THREE.Plane(
      new THREE.Vector3(0, 0, -1),
      config.ceiling.value,
  );
  const centre = new THREE.Vector3(...config.presets.initial.centre);

  let azimuth = config.presets.initial.azimuth;
  let elevation = config.presets.initial.elevation;
  let distance = config.presets.initial.distance;
  let mode = 'cloud';
  let cloud = null;
  let octomap = null;
  let history = null;
  let drag = null;
  let busy = false;
  const routeObjects = [];
  const color = new THREE.Color();

  function draw() {
    camera.position.set(
        centre.x + distance * Math.cos(elevation) * Math.cos(azimuth),
        centre.y + distance * Math.cos(elevation) * Math.sin(azimuth),
        centre.z + distance * Math.sin(elevation),
    );
    camera.lookAt(centre);
    renderer.render(scene, camera);
  }

  new ResizeObserver(() => {
    renderer.setSize(host.clientWidth, host.clientHeight);
    camera.aspect = host.clientWidth / host.clientHeight;
    camera.updateProjectionMatrix();
    draw();
  }).observe(host);

  function heightColor(z) {
    const value = THREE.MathUtils.clamp((z + 0.6) / 4.1, 0, 1);
    return color.setHSL(
        0.56 - value * 0.35,
        0.23 + value * 0.09,
        0.53 + value * 0.18,
    );
  }

  async function loadCloud() {
    if (cloud) return;
    const points = new Float32Array(
        await read(dataUrl(config.files.cloud), 'arrayBuffer'),
    );
    const colors = new Float32Array(points.length);
    for (let index = 0; index < points.length; index += 3) {
      heightColor(points[index + 2]);
      colors[index] = color.r;
      colors[index + 1] = color.g;
      colors[index + 2] = color.b;
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    cloud = new THREE.Points(
        geometry,
        new THREE.PointsMaterial({
          vertexColors: true,
          size: 0.026,
          sizeAttenuation: true,
          clippingPlanes: [plane],
        }),
    );
    cloud.visible = false;
    scene.add(cloud);
  }

  async function loadOctomap() {
    if (octomap) return;
    const values = new Float32Array(
        await read(dataUrl(config.files.occupancy), 'arrayBuffer'),
    );
    octomap = new THREE.InstancedMesh(
        new THREE.BoxGeometry(1, 1, 1),
        new THREE.MeshBasicMaterial({clippingPlanes: [plane]}),
        values.length / 5,
    );
    const matrix = new THREE.Matrix4();
    for (let index = 0; index < octomap.count; index += 1) {
      const offset = index * 5;
      const size = values[offset + 3];
      matrix.makeScale(size, size, size);
      matrix.setPosition(
          values[offset],
          values[offset + 1],
          values[offset + 2],
      );
      octomap.setMatrixAt(index, matrix);
      octomap.setColorAt(index, heightColor(values[offset + 2]));
    }
    octomap.instanceMatrix.needsUpdate = true;
    octomap.instanceColor.needsUpdate = true;
    octomap.visible = false;
    scene.add(octomap);
  }

  async function select(next) {
    if (busy) return;
    busy = true;
    $('cloud').disabled = true;
    $('octo').disabled = true;
    $('loading').textContent =
        next === 'cloud' ? '正在读取点云…' : '正在读取占据地图…';
    $('loading').hidden = false;
    $('error').hidden = true;

    try {
      await (next === 'cloud' ? loadCloud() : loadOctomap());
      mode = next;
      if (cloud) cloud.visible = mode === 'cloud';
      if (octomap) octomap.visible = mode === 'octo';
      $('cloud').setAttribute('aria-pressed', mode === 'cloud');
      $('octo').setAttribute('aria-pressed', mode === 'octo');
      $('status').textContent = mode === 'cloud' ?
          `${cloud.geometry.attributes.position.count.toLocaleString()} 点` :
          `${octomap.count.toLocaleString()} 占据体素`;
      draw();
    } finally {
      busy = false;
      $('cloud').disabled = false;
      $('octo').disabled = false;
      $('loading').hidden = true;
    }
  }

  function setPreset(preset) {
    centre.set(...preset.centre);
    distance = preset.distance;
    elevation = preset.elevation;
    if (preset.azimuth !== undefined) azimuth = preset.azimuth;
    $('top').setAttribute('aria-pressed', 'false');
    draw();
  }

  function setDetails(open) {
    $('details').hidden = !open;
    $('info').setAttribute('aria-expanded', open);
  }

  function line(points, tint, parent = scene) {
    const object = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(
            points.map((point) => new THREE.Vector3(...point)),
            ),
        new THREE.LineBasicMaterial({color: tint, depthTest: false}),
    );
    object.renderOrder = 5;
    parent.add(object);
    return object;
  }

  async function loadEvidence() {
    const hasHistory = Boolean(config.files.poses);
    const hasRoutes = Boolean(config.files.plans);
    $('history-row').hidden = !hasHistory;
    $('routes-block').hidden = !hasRoutes;
    $('evidence-group').hidden = !hasHistory && !hasRoutes;

    if (hasHistory) {
      history = line(
          await read(dataUrl(config.files.poses), 'json'),
          '#cbb58f',
      );
      history.visible = $('history').checked;
    }

    if (hasRoutes) {
      const plans = await read(dataUrl(config.files.plans), 'json');
      $('results').replaceChildren();
      for (const item of plans.cases) {
        const text = config.routeLabels?.[item.label] || item.label;
        const row = document.createElement('div');
        row.className = 'result-row';
        const label = document.createElement('span');
        label.textContent = text;
        const result = document.createElement('span');
        result.className = item.result?.ok ? 'passed' : 'failed';
        result.textContent = item.result?.ok ?
            `${item.length_m.toFixed(1)} m · ${item.wall_s.toFixed(1)} s` :
            '未通过';
        result.title =
            item.result?.ok ? '离线规划通过' : item.result?.error || '未通过';
        row.append(label, result);
        $('results').appendChild(row);

        const group = new THREE.Group();
        group.visible = false;
        scene.add(group);
        for (const [index, point] of [item.start, item.goal].entries()) {
          const geometry = index === 0 ?
              new THREE.SphereGeometry(0.12, 8, 6) :
              new THREE.BoxGeometry(0.24, 0.24, 0.24);
          const marker = new THREE.Mesh(
              geometry,
              new THREE.MeshBasicMaterial({
                color: item.result?.ok ? '#d2b0bc' : '#dc9e7a',
                depthTest: false,
              }),
          );
          marker.position.set(...point);
          marker.renderOrder = 7;
          group.add(marker);
        }
        if (item.result?.ok) line(item.result.path, '#d2b0bc', group);

        const option = document.createElement('option');
        option.value = String(routeObjects.length);
        option.textContent = text;
        $('routes').appendChild(option);
        routeObjects.push(group);
      }
      $('routes').dispatchEvent(new Event('change'));
    }
    draw();
  }

  $('cloud').onclick = () => select('cloud').catch(showError);
  $('octo').onclick = () => select('octo').catch(showError);
  ceiling.oninput = (event) => {
    plane.constant = Number(event.target.value);
    $('zout').textContent = plane.constant >= config.ceiling.max ?
        '全部' :
        `${plane.constant.toFixed(1)} m`;
    draw();
  };
  $('zout').textContent = `${plane.constant.toFixed(1)} m`;
  $('fit').onclick = () => setPreset(config.presets.full);
  $('top').onclick = () => {
    elevation = elevation > 1.55 ? config.presets.initial.elevation : 1.565;
    $('top').setAttribute('aria-pressed', elevation > 1.55);
    draw();
  };
  $('focus').onclick = () => setPreset(config.presets.focus);
  $('info').onclick = () => setDetails($('details').hidden);
  $('history').onchange = (event) => {
    if (history) history.visible = event.target.checked;
    draw();
  };
  $('routes').onchange = (event) => {
    routeObjects.forEach((object, index) => {
      object.visible =
          event.target.value === 'all' || event.target.value === String(index);
    });
    draw();
  };

  document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape' && !$('details').hidden) {
      setDetails(false);
      $('info').focus();
    }
  });

  host.onpointerdown = (event) => {
    drag = {x: event.clientX, y: event.clientY};
    host.setPointerCapture(event.pointerId);
  };

  host.onpointermove = (event) => {
    if (!drag) return;
    const deltaX = event.clientX - drag.x;
    const deltaY = event.clientY - drag.y;
    drag = {x: event.clientX, y: event.clientY};
    if (event.shiftKey) {
      const scale = distance / 750;
      centre.x +=
          (-Math.sin(azimuth) * deltaX + Math.cos(azimuth) * deltaY) * scale;
      centre.y +=
          (Math.cos(azimuth) * deltaX + Math.sin(azimuth) * deltaY) * scale;
    } else {
      azimuth -= deltaX * 0.006;
      elevation = THREE.MathUtils.clamp(
          elevation + deltaY * 0.006,
          0.04,
          1.565,
      );
      $('top').setAttribute('aria-pressed', elevation > 1.55);
    }
    draw();
  };

  host.onpointerup = host.onpointercancel = () => {
    drag = null;
  };

  host.addEventListener(
      'wheel',
      (event) => {
        event.preventDefault();
        distance = THREE.MathUtils.clamp(
            distance * Math.exp(event.deltaY * 0.001),
            0.5,
            200,
        );
        draw();
      },
      {passive: false},
  );

  window.octomapView = {
    setLayer: select,
    draw,
    get counts() {
      return {
        octomap: octomap?.count,
        cloud: cloud?.geometry.attributes.position.count,
      };
    },
    get state() {
      return {
        mode,
        ceiling: plane.constant,
        distance,
        elevation,
        azimuth,
        historyVisible: history?.visible,
        routeCount: routeObjects.length,
        visibleRoutes: routeObjects.filter((object) => object.visible).length,
      };
    },
  };

  await select('cloud').catch(showError);
  await loadEvidence().catch(showError);
}

main().catch(showError);
