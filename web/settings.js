// The settings page: changes the robot code's real constants in this sim, live (SimSettings.java lists them).
// What you change is remembered in this browser, and put back when the sim starts again. It never touches the
// real robot, so the page lists what's different in a form you can copy into RobotConstants.

const STORE = 'simSettings';

function loadSaved() {
  try {
    return JSON.parse(window.localStorage.getItem(STORE)) || {};
  } catch {
    return {};
  }
}

function save(saved) {
  try {
    window.localStorage.setItem(STORE, JSON.stringify(saved));
  } catch { /* fine, it just won't be remembered */ }
}

const send = (name, value) => fetch(`cmd?action=set&name=${encodeURIComponent(name)}&value=${value}`, { method: 'POST' }).catch(() => {});
const decimals = (step) => (String(step).split('.')[1] || '').length;
const same = (a, b) => Math.abs(a - b) < 1e-9;

export async function setUpSettings($) {
  const page = $('settings');
  let list = [];
  const saved = loadSaved();

  const show = (s, row) => {
    const changed = !same(s.value, s.default);
    row.classList.toggle('changed', changed);
    const text = row.querySelector('.settingValue');
    if (text) text.textContent = s.value.toFixed(decimals(s.step));
    row.querySelector('.settingReset').disabled = !changed;
    if (s.toggle) row.querySelector('input').checked = s.value !== 0;
    else row.querySelector('input').value = s.value;
  };

  const showChanges = () => {
    const changes = list.filter((s) => !same(s.value, s.default));
    const forRobot = changes.filter((s) => !s.simOnly);
    $('settingsChanges').value = forRobot.map((s) => {
      const [group, field] = s.name.split('.');
      const value = s.toggle ? String(s.value !== 0) : String(Number(s.value.toFixed(decimals(s.step) + 1)));
      return `${group}: ${field} = ${value};`;
    }).join('\n');
    $('settingsChanges').placeholder = 'Nothing changed from RobotConstants';
    $('settingsCount').textContent = changes.length ? `${changes.length} changed` : 'All defaults';
    $('settingsResetAll').disabled = changes.length === 0;
  };

  const change = (s, row, value) => {
    s.value = value;
    if (same(value, s.default)) delete saved[s.name];
    else saved[s.name] = value;
    save(saved);
    send(s.name, value);
    show(s, row);
    showChanges();
  };

  const build = () => {
    const body = $('settingsBody');
    body.textContent = '';
    let group = null;
    for (const s of list) {
      if (s.group !== group) {
        group = s.group;
        const title = document.createElement('h3');
        title.textContent = group;
        body.append(title);
      }
      const row = document.createElement('div');
      row.className = 'settingRow';
      const id = `set-${s.name}`;

      const label = document.createElement('label');
      label.htmlFor = id;
      label.textContent = s.label;
      row.append(label);

      const input = document.createElement('input');
      input.id = id;
      input.type = s.toggle ? 'checkbox' : 'range';
      if (!s.toggle) {
        input.min = s.min;
        input.max = s.max;
        input.step = s.step;
      }
      input.addEventListener('input', () => change(s, row, s.toggle ? (input.checked ? 1 : 0) : Number(input.value)));
      // so the arrow keys and space go back to driving the robot
      input.addEventListener('change', () => input.blur());
      row.append(input);

      if (!s.toggle) {
        const value = document.createElement('span');
        value.className = 'settingValue';
        row.append(value);
      }

      const reset = document.createElement('button');
      reset.type = 'button';
      reset.className = 'settingReset';
      reset.textContent = 'Reset';
      reset.setAttribute('aria-label', `Reset ${s.label}`);
      reset.addEventListener('click', () => change(s, row, s.default));
      row.append(reset);

      if (s.help) {
        const help = document.createElement('div');
        help.className = 'settingHelp';
        help.textContent = s.help;
        row.append(help);
      }
      s.row = row;
      body.append(row);
      show(s, row);
    }
    showChanges();
  };

  const open = (on) => {
    page.hidden = !on;
    if (on) $('settingsClose').focus();
    else $('settingsOpen').focus();
  };
  $('settingsOpen').addEventListener('click', () => open(page.hidden));
  $('settingsClose').addEventListener('click', () => open(false));
  window.addEventListener('keydown', (e) => {
    if (e.code === 'Escape' && !page.hidden) open(false);
  });
  $('settingsResetAll').addEventListener('click', () => {
    for (const s of list) {
      if (!same(s.value, s.default)) change(s, s.row, s.default);
    }
  });
  $('settingsCopy').addEventListener('click', async () => {
    try {
      await navigator.clipboard.writeText($('settingsChanges').value);
      $('settingsCopy').textContent = 'Copied';
    } catch {
      $('settingsChanges').select();
      $('settingsCopy').textContent = 'Press Ctrl+C';
    }
    setTimeout(() => { $('settingsCopy').textContent = 'Copy'; }, 1500);
  });

  try {
    const response = await fetch('cmd?action=settings', { method: 'POST' });
    list = await response.json();
  } catch {
    return; // the sim's not running, the button just opens an empty page
  }
  // a new sim starts from RobotConstants, put back what this browser had
  for (const s of list) {
    if (saved[s.name] === undefined) continue;
    s.value = Math.max(s.min, Math.min(s.max, Number(saved[s.name])));
    send(s.name, s.value);
  }
  build();
}
