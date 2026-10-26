(() => {
  'use strict';
  const data = window.CONPR_BENCHMARK;
  if (!Array.isArray(data) || data.length !== 228) return;

  const methods = {
    LiDAR: ['SOLiD', 'UniMPR-L', 'CASSPR', 'BEVPlace++'],
    Visual: ['SALAD', 'UniMPR-C', 'SelaVPR++', 'ImAge'],
    Fusion: ['SALAD+CASSPR', 'UniMPR-LC', 'MinkLoc++', 'UMF'],
  };
  const metrics = {r1: 'Recall@1 (%)', auc: 'PR-AUC', f1: 'Maximum F1'};
  const site = document.getElementById('result-site');
  const modality = document.getElementById('result-modality');
  const metric = document.getElementById('result-metric');
  const table = document.getElementById('result-table');

  function cell(tag, content, className = '') {
    const element = document.createElement(tag);
    element.textContent = content;
    if (className) element.className = className;
    return element;
  }

  function renderResults() {
    const selected = data.filter(row => row.site === site.value && row.modality === modality.value);
    const databases = [...new Set(selected.map(row => row.db))].sort().reverse();
    const columns = methods[modality.value];
    const caption = cell('caption', `${site.value}: ${modality.value} ${metrics[metric.value]}`);
    const head = document.createElement('thead');
    const header = document.createElement('tr');
    ['DB → Query', 'TCR_VG', ...columns].forEach((name, index) => {
      const th = cell('th', name, index ? 'numeric' : '');
      th.scope = 'col';
      header.append(th);
    });
    head.append(header);
    const body = document.createElement('tbody');
    for (const db of databases) {
      const rows = columns.map(method => selected.find(row => row.db === db && row.method === method));
      const values = rows.map(row => row[metric.value]);
      const ranks = [];
      for (const value of [...values].sort((a, b) => b - a)) {
        if (!ranks.some(rank => Math.abs(rank - value) <= 1e-12)) ranks.push(value);
      }
      const tr = document.createElement('tr');
      const night = (site.value === 'Hall' && db === '03') || (site.value === 'Office' && ['02', '11'].includes(db));
      const label = cell('th', `${db}${night ? '†' : ''} → ${rows[0].query}`);
      label.scope = 'row';
      if (night) label.setAttribute('aria-label', `Database ${db}, night, to query ${rows[0].query}`);
      tr.append(label, cell('td', rows[0].tcr.toFixed(3), 'numeric change-score'));
      values.forEach(value => {
        const td = document.createElement('td');
        td.className = 'numeric';
        const formatted = metric.value === 'r1' ? (value * 100).toFixed(2) : value.toFixed(4);
        const best = Math.abs(value - ranks[0]) <= 1e-12;
        const second = ranks.length > 1 && Math.abs(value - ranks[1]) <= 1e-12;
        td.append(cell(best ? 'strong' : 'span', formatted, second ? 'runner-up' : ''));
        if (best || second) td.setAttribute('aria-label', `${formatted}, ${best ? 'best' : 'second-best'}`);
        tr.append(td);
      });
      body.append(tr);
    }
    table.replaceChildren(caption, head, body);
    document.getElementById('result-status').textContent = `${site.value} · ${modality.value} · ${metrics[metric.value]} · ${databases.length} session pairs`;
    document.getElementById('result-scope').textContent = metric.value === 'r1'
      ? 'R@1 is evaluated on queries with at least one positive database sample (Q+). Values are percentages; higher is better.'
      : `${metrics[metric.value]} is evaluated over all query–database pairs, including queries without positives. Values are on a 0–1 scale; higher is better.`;
  }
  [site, modality, metric].forEach(control => control.addEventListener('change', renderResults));
  renderResults();
  document.getElementById('result-controls').hidden = false;

  function setupFigures(prefix) {
    const select = document.getElementById(`${prefix}-modality`);
    const figures = document.querySelectorAll(`#${prefix}-figures [data-modality]`);
    function update() {
      figures.forEach(figure => { figure.hidden = figure.dataset.modality !== select.value; });
    }
    select.addEventListener('change', update);
    update();
    document.getElementById(`${prefix}-controls`).hidden = false;
  }
  setupFigures('pr');
  setupFigures('change');
})();
