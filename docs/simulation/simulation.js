/**
 * RFS Gradient Descent Simulation
 * Logic based on FACES IV Model documentation
 * Strictly synchronized with RFSTherapist node visual coordinates
 */

const ctx = document.getElementById('circumplexChart').getContext('2d');
let chart;
let animationId = null;
let isRunning = false;

// State Variables (Current)
let state = {
    c_bal: 20, c_dis: 80, c_enm: 10,
    f_bal: 20, f_rig: 80, f_cha: 10,
    comm: 50
};

// Values at startup or set by manual slider intervention
let startingState = JSON.parse(JSON.stringify(state));

let weights = {
    w1: 1.0, w2: 1.0, w3: 2.0
};

let lr_scale = 0.25;
let pathHistory = []; // Visual coordinates {x, y} for the chart
let stateHistory = []; // Full state objects for undo/backward

const tick_vals = [0, 8, 15, 16, 25, 35, 36, 50, 65, 66, 75, 85, 86, 95, 100];

/**
 * --- Piecewise Linear Visual Mapping ---
 * Maps 0-100 FACES scores to 0.0 - 5.0 visual coordinates.
 * Strictly matches RFSTherapist's get_visual_coord logic.
 */
function getVisualCoord(score) {
    const gap = 0.04;
    const ranges = [
        { idx: 0, min: 0, mid: 8, max: 15 },
        { idx: 1, min: 16, mid: 25, max: 35 },
        { idx: 2, min: 36, mid: 50, max: 65 },
        { idx: 3, min: 66, mid: 75, max: 85 },
        { idx: 4, min: 86, mid: 95, max: 100 }
    ];

    let r = ranges.find(range => score >= range.min && score <= range.max);
    if (!r) {
        if (score < 0) r = ranges[0];
        else r = ranges[4];
    }

    const vis_start = r.idx + gap;
    const vis_mid = r.idx + 0.5;
    const vis_end = r.idx + 1.0 - gap;

    if (score <= r.mid) {
        if (r.mid === r.min) return vis_start;
        const norm = (score - r.min) / (r.mid - r.min);
        return vis_start + norm * (vis_mid - vis_start);
    } else {
        if (r.max === r.mid) return vis_end;
        const norm = (score - r.mid) / (r.max - r.mid);
        return vis_mid + norm * (vis_end - vis_mid);
    }
}

/**
 * Syncs the internal state with the ACTUAL values currently in the sliders.
 * Used at boot and before simulation starts.
 */
function syncFromSliders() {
    Object.keys(state).forEach(key => {
        const el = document.getElementById(key);
        if (el) state[key] = parseFloat(el.value);
    });
    Object.keys(weights).forEach(key => {
        const el = document.getElementById(key);
        if (el) weights[key] = parseFloat(el.value);
    });
    const lrEl = document.getElementById('lr_scale');
    if (lrEl) lr_scale = parseFloat(lrEl.value);

    // Update the starting point for "Reset"
    startingState = JSON.parse(JSON.stringify(state));

    // Calculate current x, y
    const x = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const y = state.f_bal + (state.f_cha - state.f_rig) / 2;
    const vx = getVisualCoord(x);
    const vy = getVisualCoord(y);

    pathHistory = [{ x: vx, y: vy }];
    stateHistory = [{ state: JSON.parse(JSON.stringify(state)), x, y }];
}

/**
 * Reverts the UI and chart to the start of the simulation.
 */
function init() {
    // Revert state to the point where simulation was reset/booted
    state = JSON.parse(JSON.stringify(startingState));

    updateUI();
    const x = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const y = state.f_bal + (state.f_cha - state.f_rig) / 2;
    const vx = getVisualCoord(x);
    const vy = getVisualCoord(y);

    pathHistory = [{ x: vx, y: vy }];
    stateHistory = [{ state: JSON.parse(JSON.stringify(state)), x, y }];

    if (chart) chart.destroy();
    createChart();

    updateMetrics();
    updateButtonStates();
}

function updateUI() {
    Object.keys(state).forEach(key => {
        const valEl = document.getElementById(key + '_val');
        if (valEl) valEl.innerText = state[key].toFixed(0);
    });
    Object.keys(weights).forEach(key => {
        const valEl = document.getElementById(key + '_val');
        if (valEl) valEl.innerText = weights[key].toFixed(1);
    });
    const lrValEl = document.getElementById('lr_scale_val');
    if (lrValEl) lrValEl.innerText = lr_scale.toFixed(2);
}

function updateMetrics() {
    const coh_unbal = (state.c_dis + state.c_enm) / 2;
    const flex_unbal = (state.f_rig + state.f_cha) / 2;

    const coh_ratio = state.c_bal / Math.max(1, coh_unbal);
    const flex_ratio = state.f_bal / Math.max(1, flex_unbal);
    const total_ratio = (state.c_bal + state.f_bal) / Math.max(1, coh_unbal + flex_unbal);

    const last = stateHistory[stateHistory.length - 1];
    document.getElementById('current_pos').innerText = `${Math.round(last.x)}, ${Math.round(last.y)}`;
    document.getElementById('coh_ratio').innerText = coh_ratio.toFixed(2);
    document.getElementById('flex_ratio').innerText = flex_ratio.toFixed(2);
    document.getElementById('total_ratio').innerText = total_ratio.toFixed(2);
}

function updateButtonStates() {
    const prevBtn = document.getElementById('prev-btn');
    if (prevBtn) {
        prevBtn.disabled = stateHistory.length <= 1;
    }
}

function updateRunButtonText() {
    const btn = document.getElementById('run-btn');
    if (btn) {
        btn.innerText = isRunning ? 'Pause Simulation' : 'Play Simulation';
    }
}

// Math logic
function step() {
    const B = Math.max(0.1, state.c_bal + state.f_bal);
    const U = state.c_dis + state.c_enm + state.f_rig + state.f_cha;
    const x = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const y = state.f_bal + (state.f_cha - state.f_rig) / 2;

    const eta = Math.max(0.15, state.comm / 100) * lr_scale;

    const grad_bal_prefix = - (weights.w1 * U) / (2.0 * B ** 2);
    const dJ_dc_bal = grad_bal_prefix + weights.w3 * (x - 50.0);
    const dJ_df_bal = grad_bal_prefix + weights.w3 * (y - 50.0);

    const grad_unbal_prefix = weights.w1 / (2.0 * B);
    const dJ_dc_enm = grad_unbal_prefix + (weights.w3 / 2.0) * (x - 50.0);
    const dJ_dc_dis = grad_unbal_prefix - (weights.w3 / 2.0) * (x - 50.0);
    const dJ_df_cha = grad_unbal_prefix + (weights.w3 / 2.0) * (y - 50.0);
    const dJ_df_rig = grad_unbal_prefix - (weights.w3 / 2.0) * (y - 50.0);

    const dJ_dcomm = - weights.w2;

    let delta = {
        c_bal: -eta * dJ_dc_bal,
        f_bal: -eta * dJ_df_bal,
        c_enm: -eta * dJ_dc_enm,
        c_dis: -eta * dJ_dc_dis,
        f_cha: -eta * dJ_df_cha,
        f_rig: -eta * dJ_df_rig,
        comm: -eta * dJ_dcomm
    };

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

    if (alpha < 1.0) {
        Object.keys(delta).forEach(k => delta[k] *= alpha);
    }

    state.c_bal = Math.min(100, Math.max(0, state.c_bal + delta.c_bal));
    state.f_bal = Math.min(100, Math.max(0, state.f_bal + delta.f_bal));
    state.c_enm = Math.min(100, Math.max(0, state.c_enm + delta.c_enm));
    state.c_dis = Math.min(100, Math.max(0, state.c_dis + delta.c_dis));
    state.f_cha = Math.min(100, Math.max(0, state.f_cha + delta.f_cha));
    state.f_rig = Math.min(100, Math.max(0, state.f_rig + delta.f_rig));
    state.comm = Math.min(100, Math.max(0, state.comm + delta.comm));

    const nextX = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const nextY = state.f_bal + (state.f_cha - state.f_rig) / 2;

    pathHistory.push({ x: getVisualCoord(nextX), y: getVisualCoord(nextY) });
    stateHistory.push({ state: JSON.parse(JSON.stringify(state)), x: nextX, y: nextY });

    if (pathHistory.length > 500) {
        pathHistory.shift();
        stateHistory.shift();
    }

    updateUI();
    updateMetrics();
    updateChart();
    updateButtonStates();

    if (isRunning) {
        animationId = setTimeout(step, 100);
    }
}

function undoStep() {
    if (stateHistory.length > 1) {
        stateHistory.pop();
        pathHistory.pop();
        state = JSON.parse(JSON.stringify(stateHistory[stateHistory.length - 1].state));

        updateUI();
        updateMetrics();
        updateChart();
        updateButtonStates();
    }
}

// Charting
function createChart() {
    const gridAnnotations = {};
    for (let i = 0; i < 5; i++) {
        for (let j = 0; j < 5; j++) {
            let color = '#d3d3d3'; // Default: Light Gray
            if ((i === 0 && j === 0) || (i === 0 && j === 4) || (i === 4 && j === 0) || (i === 4 && j === 4)) {
                color = '#a9a9a9'; // Corners: Gray
            } else if (i >= 1 && i <= 3 && j >= 1 && j <= 3) {
                color = '#ffffff'; // Center: White
            }
            gridAnnotations[`grid_${i}_${j}`] = {
                type: 'box',
                xMin: i + 0.04, xMax: i + 0.96,
                yMin: j + 0.04, yMax: j + 0.96,
                backgroundColor: color,
                borderColor: '#18181b',
                borderWidth: 1.5,
                drawTime: 'beforeDraw'
            };
        }
    }

    chart = new Chart(ctx, {
        type: 'scatter',
        data: {
            datasets: [
                {
                    label: 'Path',
                    data: pathHistory,
                    borderColor: '#0ea5e9',
                    showLine: true,
                    borderWidth: 3,
                    pointRadius: 0,
                    tension: 0.1
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: true,
            aspectRatio: 1,
            layout: { padding: { left: 10, right: 30, top: 10, bottom: 20 } },
            scales: {
                x: {
                    min: 0, max: 5,
                    title: { display: true, text: 'COHESION', color: '#94a3b8' },
                    grid: { display: false },
                    ticks: {
                        color: '#71717a',
                        autoSkip: false,
                        maxRotation: 0,
                        callback: function (value) {
                            for (let v of tick_vals) {
                                if (Math.abs(getVisualCoord(v) - value) < 0.0005) return v;
                            }
                            return null;
                        },
                        stepSize: 0.0001
                    }
                },
                y: {
                    min: 0, max: 5,
                    title: { display: true, text: 'FLEXIBILITY', color: '#94a3b8' },
                    grid: { display: false },
                    ticks: {
                        color: '#71717a',
                        autoSkip: false,
                        maxRotation: 0,
                        callback: function (value) {
                            for (let v of tick_vals) {
                                if (Math.abs(getVisualCoord(v) - value) < 0.0005) return v;
                            }
                            return null;
                        },
                        stepSize: 0.0001
                    }
                }
            },
            plugins: {
                annotation: {
                    annotations: {
                        ...gridAnnotations,
                        balancedLabel: {
                            type: 'label',
                            xValue: 2.5, yValue: 2.5,
                            content: 'BALANCED',
                            color: 'rgba(34, 197, 94, 0.2)',
                            font: { size: 32, weight: 'bold' },
                            drawTime: 'beforeDraw'
                        },
                        centerTarget: {
                            type: 'point',
                            xValue: getVisualCoord(50), yValue: getVisualCoord(50),
                            backgroundColor: 'rgba(0, 0, 0, 0.1)',
                            radius: 4,
                            drawTime: 'beforeDraw'
                        },
                        currentPosMarker: {
                            type: 'point',
                            xValue: pathHistory[0].x,
                            yValue: pathHistory[0].y,
                            backgroundColor: '#ef4444',
                            borderColor: '#ffffff',
                            borderWidth: 3,
                            radius: 10,
                            z: 200 // TOP LAYER
                        }
                    }
                },
                legend: { display: false },
                tooltip: { enabled: false }
            }
        }
    });
}

function updateChart() {
    chart.data.datasets[0].data = pathHistory;
    const lastPoint = pathHistory[pathHistory.length - 1];
    chart.options.plugins.annotation.annotations.currentPosMarker.xValue = lastPoint.x;
    chart.options.plugins.annotation.annotations.currentPosMarker.yValue = lastPoint.y;
    chart.update('none');
}

// Event Listeners
document.querySelectorAll('input[type="range"]').forEach(slider => {
    slider.addEventListener('input', (e) => {
        const id = e.target.id;
        const val = parseFloat(e.target.value);

        // Update current local values
        if (state[id] !== undefined) state[id] = val;
        if (weights[id] !== undefined) weights[id] = val;
        if (id === 'lr_scale') lr_scale = val;

        // UI Text
        const valEl = document.getElementById(id + '_val');
        if (valEl) valEl.innerText = id.startsWith('w') || id === 'lr_scale' ? val.toFixed(1) : val.toFixed(0);

        // If simulation is NOT running, this becomes the start of the next run
        if (!isRunning) {
            syncFromSliders(); // This also updates startingState and calculates history
            updateChart();
            updateMetrics();
            updateButtonStates();
        }
    });
});

document.getElementById('run-btn').addEventListener('click', () => {
    isRunning = !isRunning;
    updateRunButtonText();
    if (isRunning) step();
});

document.getElementById('reset-btn').addEventListener('click', () => {
    isRunning = false;
    clearTimeout(animationId);
    updateRunButtonText();
    init(); // Reverts to startingState
});

document.getElementById('next-btn').addEventListener('click', () => {
    if (isRunning) {
        isRunning = false;
        clearTimeout(animationId);
        updateRunButtonText();
    }
    step();
});

document.getElementById('prev-btn').addEventListener('click', () => {
    if (isRunning) {
        isRunning = false;
        clearTimeout(animationId);
        updateRunButtonText();
    }
    if (stateHistory.length > 1) undoStep();
});

// STARTUP: Sync from HTML defaults first
syncFromSliders();
init();
