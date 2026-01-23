/**
 * RFS Gradient Descent Simulation
 * Logic based on FACES IV Model documentation
 */

const ctx = document.getElementById('circumplexChart').getContext('2d');
let chart;
let animationId = null;
let isRunning = false;

// State Variables
let state = {
    c_bal: 20, c_dis: 80, c_enm: 10,
    f_bal: 20, f_rig: 80, f_cha: 10,
    comm: 50
};

let weights = {
    w1: 1.0, w2: 1.0, w3: 2.0
};

let lr_scale = 0.25;
let pathHistory = []; // Coordinates for the chart
let stateHistory = []; // Full state objects for undo/backward

// Initialize Values
function init() {
    state = {
        c_bal: 20, c_dis: 80, c_enm: 10,
        f_bal: 20, f_rig: 80, f_cha: 10,
        comm: 50
    };
    lr_scale = 0.25;

    updateUI();
    const startX = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const startY = state.f_bal + (state.f_cha - state.f_rig) / 2;
    pathHistory = [{ x: startX, y: startY }];
    stateHistory = [JSON.parse(JSON.stringify(state))];

    if (chart) {
        updateChart();
    } else {
        createChart();
    }
    updateMetrics();
}

function updateUI() {
    Object.keys(state).forEach(key => {
        const el = document.getElementById(key);
        if (el) el.value = state[key];
        const valEl = document.getElementById(key + '_val');
        if (valEl) valEl.innerText = state[key].toFixed(0);
    });
    Object.keys(weights).forEach(key => {
        const el = document.getElementById(key);
        if (el) el.value = weights[key];
        const valEl = document.getElementById(key + '_val');
        if (valEl) valEl.innerText = weights[key].toFixed(1);
    });
    document.getElementById('lr_scale').value = lr_scale;
    document.getElementById('lr_scale_val').innerText = lr_scale.toFixed(2);
}

function updateMetrics() {
    const coh_ratio = state.c_bal / ((state.c_dis + state.c_enm) / 2);
    const flex_ratio = state.f_bal / ((state.f_rig + state.f_cha) / 2);
    const total_ratio = (state.c_bal + state.f_bal) / ((state.c_dis + state.c_enm + state.f_rig + state.f_cha) / 2);

    const x = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const y = state.f_bal + (state.f_cha - state.f_rig) / 2;

    document.getElementById('current_pos').innerText = `${Math.round(x)}, ${Math.round(y)}`;
    document.getElementById('coh_ratio').innerText = coh_ratio.toFixed(2);
    document.getElementById('flex_ratio').innerText = flex_ratio.toFixed(2);
    document.getElementById('total_ratio').innerText = total_ratio.toFixed(2);
}

// Math logic
function step() {
    const B = Math.max(0.1, state.c_bal + state.f_bal);
    const U = state.c_dis + state.c_enm + state.f_rig + state.f_cha;
    const x = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const y = state.f_bal + (state.f_cha - state.f_rig) / 2;

    // Learning rate eta (Strictly faithful to therapist node: max(0.15, comm/100) * scaling)
    const eta = Math.max(0.15, state.comm / 100) * lr_scale;

    // Gradients: J = w1*(U/2B) - w2*Comm + w3*0.5*((x-50)^2 + (y-50)^2)
    const grad_bal_prefix = - (weights.w1 * U) / (2.0 * B ** 2);
    const dJ_dc_bal = grad_bal_prefix + weights.w3 * (x - 50.0);
    const dJ_df_bal = grad_bal_prefix + weights.w3 * (y - 50.0);

    const grad_unbal_prefix = weights.w1 / (2.0 * B);
    const dJ_dc_enm = grad_unbal_prefix + (weights.w3 / 2.0) * (x - 50.0);
    const dJ_dc_dis = grad_unbal_prefix - (weights.w3 / 2.0) * (x - 50.0);
    const dJ_df_cha = grad_unbal_prefix + (weights.w3 / 2.0) * (y - 50.0);
    const dJ_df_rig = grad_unbal_prefix - (weights.w3 / 2.0) * (y - 50.0);

    const dJ_dcomm = - weights.w2;

    // Proposed updates
    let delta = {
        c_bal: -eta * dJ_dc_bal,
        f_bal: -eta * dJ_df_bal,
        c_enm: -eta * dJ_dc_enm,
        c_dis: -eta * dJ_dc_dis,
        f_cha: -eta * dJ_df_cha,
        f_rig: -eta * dJ_df_rig,
        comm: -eta * dJ_dcomm
    };

    // Adjacency constraint (Matches calculate_gradient in therapist node)
    function getCellIdx(v) {
        if (v <= 15) return 0;
        if (v <= 35) return 1;
        if (v <= 65) return 2;
        if (v <= 85) return 3;
        return 4;
    }

    const nextXRaw = (state.c_bal + delta.c_bal) + ((state.c_enm + delta.c_enm) - (state.c_dis + delta.c_dis)) / 2.0;
    const nextYRaw = (state.f_bal + delta.f_bal) + ((state.f_cha + delta.f_cha) - (state.f_rig + delta.f_rig)) / 2.0;

    const currI = getCellIdx(x);
    const currJ = getCellIdx(y);
    const ranges = [[0, 15], [16, 35], [36, 65], [66, 85], [86, 100]];
    const lowX = ranges[Math.max(0, currI - 1)][0];
    const highX = ranges[Math.min(4, currI + 1)][1];
    const lowY = ranges[Math.max(0, currJ - 1)][0];
    const highY = ranges[Math.min(4, currJ + 1)][1];

    let alpha = 1.0;
    const dx = nextXRaw - x;
    const dy = nextYRaw - y;
    if (dx !== 0) {
        if (x + dx > highX) alpha = Math.min(alpha, (highX - x) / dx);
        if (x + dx < lowX) alpha = Math.min(alpha, (lowX - x) / dx);
    }
    if (dy !== 0) {
        if (y + dy > highY) alpha = Math.min(alpha, (highY - y) / dy);
        if (y + dy < lowY) alpha = Math.min(alpha, (lowY - y) / dy);
    }

    // Apply alpha scaling to all deltas if constrained
    if (alpha < 1.0) {
        Object.keys(delta).forEach(k => delta[k] *= alpha);
    }

    // Final State Application
    state.c_bal = Math.min(100, Math.max(0, state.c_bal + delta.c_bal));
    state.f_bal = Math.min(100, Math.max(0, state.f_bal + delta.f_bal));
    state.c_enm = Math.min(100, Math.max(0, state.c_enm + delta.c_enm));
    state.c_dis = Math.min(100, Math.max(0, state.c_dis + delta.c_dis));
    state.f_cha = Math.min(100, Math.max(0, state.f_cha + delta.f_cha));
    state.f_rig = Math.min(100, Math.max(0, state.f_rig + delta.f_rig));
    state.comm = Math.min(100, Math.max(0, state.comm + delta.comm));

    const nextX = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const nextY = state.f_bal + (state.f_cha - state.f_rig) / 2;

    pathHistory.push({ x: nextX, y: nextY });
    stateHistory.push(JSON.parse(JSON.stringify(state)));

    if (pathHistory.length > 100) {
        pathHistory.shift();
        stateHistory.shift();
    }

    updateUI();
    updateMetrics();
    updateChart();

    if (isRunning) {
        animationId = setTimeout(step, 100);
    }
}

function undoStep() {
    if (stateHistory.length > 1) {
        stateHistory.pop();
        pathHistory.pop();
        state = JSON.parse(JSON.stringify(stateHistory[stateHistory.length - 1]));

        updateUI();
        updateMetrics();
        updateChart();
    }
}

// Charting
function createChart() {
    chart = new Chart(ctx, {
        type: 'scatter',
        data: {
            datasets: [
                {
                    label: 'Path',
                    data: pathHistory,
                    borderColor: '#38bdf8',
                    showLine: true,
                    borderWidth: 2,
                    pointRadius: 0,
                    tension: 0.1
                },
                {
                    label: 'Current Position',
                    data: [pathHistory[pathHistory.length - 1]],
                    backgroundColor: '#ef4444',
                    pointRadius: 6,
                    pointHoverRadius: 8
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    min: -50, max: 150,
                    title: { display: true, text: 'Cohesion Index (Offset from Center)', color: '#94a3b8' },
                    grid: { color: '#334155' },
                    ticks: { color: '#94a3b8' }
                },
                y: {
                    min: -50, max: 150,
                    title: { display: true, text: 'Flexibility Index (Offset from Center)', color: '#94a3b8' },
                    grid: { color: '#334155' },
                    ticks: { color: '#94a3b8' }
                }
            },
            plugins: {
                annotation: {
                    annotations: {
                        balancedRegion: {
                            type: 'box',
                            xMin: 36, xMax: 65, yMin: 36, yMax: 65,
                            backgroundColor: 'rgba(34, 197, 94, 0.1)',
                            borderColor: 'rgba(34, 197, 94, 0.5)',
                            borderWidth: 2,
                            label: { display: true, content: 'Balanced Region (Target)', color: '#22c55e', font: { size: 10 } }
                        },
                        targetPoint: {
                            type: 'point',
                            xValue: 50, yValue: 50,
                            backgroundColor: 'rgba(255, 255, 255, 0.8)',
                            radius: 4
                        }
                    }
                },
                legend: { display: false }
            }
        }
    });
}

function updateChart() {
    chart.data.datasets[0].data = pathHistory;
    chart.data.datasets[1].data = [pathHistory[pathHistory.length - 1]];
    chart.update('none');
}

// Event Listeners
document.querySelectorAll('input[type="range"]').forEach(slider => {
    slider.addEventListener('input', (e) => {
        const id = e.target.id;
        const val = parseFloat(e.target.value);
        if (state[id] !== undefined) state[id] = val;
        if (weights[id] !== undefined) weights[id] = val;
        if (id === 'lr_scale') lr_scale = val;

        const valEl = document.getElementById(id + '_val');
        if (valEl) valEl.innerText = id.startsWith('w') || id === 'lr_scale' ? val.toFixed(1) : val.toFixed(0);

        if (!isRunning) {
            const x = state.c_bal + (state.c_enm - state.c_dis) / 2;
            const y = state.f_bal + (state.f_cha - state.f_rig) / 2;
            pathHistory = [{ x, y }];
            stateHistory = [JSON.parse(JSON.stringify(state))];
            updateChart();
            updateMetrics();
        }
    });
});

document.getElementById('run-btn').addEventListener('click', () => {
    isRunning = !isRunning;
    const btn = document.getElementById('run-btn');
    btn.innerText = isRunning ? 'Pause Simulation' : 'Play Simulation';
    btn.classList.toggle('primary', isRunning);

    if (isRunning) step();
});

document.getElementById('reset-btn').addEventListener('click', () => {
    isRunning = false;
    clearTimeout(animationId);
    const btn = document.getElementById('run-btn');
    btn.innerText = 'Play Simulation';
    btn.classList.add('primary');

    init();
});

document.getElementById('next-btn').addEventListener('click', () => {
    if (!isRunning) step();
});

document.getElementById('prev-btn').addEventListener('click', () => {
    if (!isRunning) undoStep();
});

// Start
init();
