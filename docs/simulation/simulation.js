/**
 * RFS Gradient Descent Simulation
 * Logic based on FACES IV Model documentation
 * Strictly synchronized with RFSTherapist node logic and visual mapping.
 */

const ctx = document.getElementById('circumplexChart').getContext('2d');
let chart;
let animationId = null;
let isRunning = false;

// Initial configuration (tied to sliders)
let configState = {
    c_bal: 20, c_dis: 80, c_enm: 10,
    f_bal: 20, f_rig: 80, f_cha: 10,
    comm: 50
};

// Active simulation state (evolves via math)
let currentState = JSON.parse(JSON.stringify(configState));

let weights = {
    w1: 1.0, w2: 1.0, w3: 2.0
};

let lr_scale = 0.25;
let pathHistory = []; // Visual coordinates {x, y} [0-5]
let stateHistory = []; // Clipped dimension scores {state, x, y} [5-95]

const tick_vals = [0, 8, 15, 16, 25, 35, 36, 50, 65, 66, 75, 85, 86, 95, 100];

/**
 * --- Dimension Score Calculation (Percentile) ---
 * Converts 7-variable state to X (Cohesion) and Y (Flexibility) dimension scores.
 * Clipped to [5, 95] as requested.
 */
function getClippedDimensionScores(s) {
    let x = s.c_bal + (s.c_enm - s.c_dis) / 2.0;
    let y = s.f_bal + (s.f_cha - s.f_rig) / 2.0;

    // Strict Clipping to [5, 95]
    return {
        x: Math.max(5, Math.min(95, x)),
        y: Math.max(5, Math.min(95, y))
    };
}

/**
 * --- Piecewise Linear Visual Mapping ---
 * Maps 0-100 percentile scores to 0.0 - 5.0 visual coordinates.
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
 * Syncs the starting point from the slider positions.
 * Updates configState and initializes currentState.
 */
function syncFromUI() {
    // Read sliders into configState (Initial Config)
    Object.keys(configState).forEach(key => {
        const el = document.getElementById(key);
        if (el) configState[key] = parseFloat(el.value);
    });
    Object.keys(weights).forEach(key => {
        const el = document.getElementById(key);
        if (el) weights[key] = parseFloat(el.value);
    });
    const lrEl = document.getElementById('lr_scale');
    if (lrEl) lr_scale = parseFloat(lrEl.value);

    // Initialize current simulation state to match config
    currentState = JSON.parse(JSON.stringify(configState));

    // Calculate initial point
    const dim = getClippedDimensionScores(currentState);
    pathHistory = [{ x: getVisualCoord(dim.x), y: getVisualCoord(dim.y) }];
    stateHistory = [{ state: JSON.parse(JSON.stringify(currentState)), x: dim.x, y: dim.y }];

    updateMetrics();
    updateChart();
    updateButtonStates();
    updateSliderLabels(); // Always show slider values next to sliders
}

/**
 * Resets the entire simulation to the state held in the sliders.
 */
function init() {
    syncFromUI();

    if (chart) chart.destroy();
    createChart();
}

/**
 * Updates only the textual value indicators NEXT to sliders.
 * These reflect the INITIAL values set by the user.
 */
function updateSliderLabels() {
    Object.keys(configState).forEach(key => {
        const valEl = document.getElementById(key + '_val');
        if (valEl) valEl.innerText = configState[key].toFixed(0);
    });
    Object.keys(weights).forEach(key => {
        const valEl = document.getElementById(key + '_val');
        if (valEl) valEl.innerText = weights[key].toFixed(1);
    });
    const lrValEl = document.getElementById('lr_scale_val');
    if (lrValEl) lrValEl.innerText = lr_scale.toFixed(2);
}

function updateMetrics() {
    const last = stateHistory[stateHistory.length - 1];

    // Use currentState for ratios (Active/Final State)
    const coh_unbal = (currentState.c_dis + currentState.c_enm) / 2;
    const flex_unbal = (currentState.f_rig + currentState.f_cha) / 2;
    const coh_ratio = currentState.c_bal / Math.max(1, coh_unbal);
    const flex_ratio = currentState.f_bal / Math.max(1, flex_unbal);
    const total_ratio = (currentState.c_bal + currentState.f_bal) / Math.max(1, coh_unbal + flex_unbal);

    document.getElementById('current_pos').innerText = `${Math.round(last.x)}, ${Math.round(last.y)}`;
    document.getElementById('coh_ratio').innerText = coh_ratio.toFixed(2);
    document.getElementById('flex_ratio').innerText = flex_ratio.toFixed(2);
    document.getElementById('total_ratio').innerText = total_ratio.toFixed(2);
}

function updateButtonStates() {
    const prevBtn = document.getElementById('prev-btn');
    if (prevBtn) prevBtn.disabled = stateHistory.length <= 1;
}

function updateRunButtonText() {
    const btn = document.getElementById('run-btn');
    if (btn) btn.innerText = isRunning ? 'Pause Simulation' : 'Play Simulation';
}

/**
 * --- Gradient Descent Step ---
 */
function step() {
    const B = Math.max(0.1, currentState.c_bal + currentState.f_bal);
    const U = currentState.c_dis + currentState.c_enm + currentState.f_rig + currentState.f_cha;

    const rawX = currentState.c_bal + (currentState.c_enm - currentState.c_dis) / 2.0;
    const rawY = currentState.f_bal + (currentState.f_cha - currentState.f_rig) / 2.0;

    const eta = Math.max(0.15, currentState.comm / 100) * lr_scale;

    const grad_bal_prefix = - (weights.w1 * U) / (2.0 * B ** 2);
    const dJ_dc_bal = grad_bal_prefix + weights.w3 * (rawX - 50.0);
    const dJ_df_bal = grad_bal_prefix + weights.w3 * (rawY - 50.0);

    const grad_unbal_prefix = weights.w1 / (2.0 * B);
    const dJ_dc_enm = grad_unbal_prefix + (weights.w3 / 2.0) * (rawX - 50.0);
    const dJ_dc_dis = grad_unbal_prefix - (weights.w3 / 2.0) * (rawX - 50.0);
    const dJ_df_cha = grad_unbal_prefix + (weights.w3 / 2.0) * (rawY - 50.0);
    const dJ_df_rig = grad_unbal_prefix - (weights.w3 / 2.0) * (rawY - 50.0);

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

    // Adjacency Constraint
    function getCellIdx(v) {
        if (v <= 15) return 0;
        if (v <= 35) return 1;
        if (v <= 65) return 2;
        if (v <= 85) return 3;
        return 4;
    }

    const currI = getCellIdx(rawX);
    const currJ = getCellIdx(rawY);
    const ranges = [[0, 15], [16, 35], [36, 65], [66, 85], [86, 100]];
    const lowX = ranges[Math.max(0, currI - 1)][0];
    const highX = ranges[Math.min(4, currI + 1)][1];
    const lowY = ranges[Math.max(0, currJ - 1)][0];
    const highY = ranges[Math.min(4, currJ + 1)][1];

    const nextXRaw = (currentState.c_bal + delta.c_bal) + ((currentState.c_enm + delta.c_enm) - (currentState.c_dis + delta.c_dis)) / 2.0;
    const nextYRaw = (currentState.f_bal + delta.f_bal) + ((currentState.f_cha + delta.f_cha) - (currentState.f_rig + delta.f_rig)) / 2.0;

    let alpha = 1.0;
    const dx = nextXRaw - rawX;
    const dy = nextYRaw - rawY;
    if (dx !== 0) {
        if (rawX + dx > highX) alpha = Math.min(alpha, (highX - rawX) / dx);
        if (rawX + dx < lowX) alpha = Math.min(alpha, (lowX - rawX) / dx);
    }
    if (dy !== 0) {
        if (rawY + dy > highY) alpha = Math.min(alpha, (highY - rawY) / dy);
        if (rawY + dy < lowY) alpha = Math.min(alpha, (lowY - rawY) / dy);
    }

    if (alpha < 1.0) {
        Object.keys(delta).forEach(k => delta[k] *= alpha);
    }

    // Apply Deltas to active simulation state
    currentState.c_bal = Math.min(100, Math.max(0, currentState.c_bal + delta.c_bal));
    currentState.f_bal = Math.min(100, Math.max(0, currentState.f_bal + delta.f_bal));
    currentState.c_enm = Math.min(100, Math.max(0, currentState.c_enm + delta.c_enm));
    currentState.c_dis = Math.min(100, Math.max(0, currentState.c_dis + delta.c_dis));
    currentState.f_cha = Math.min(100, Math.max(0, currentState.f_cha + delta.f_cha));
    currentState.f_rig = Math.min(100, Math.max(0, currentState.f_rig + delta.f_rig));
    currentState.comm = Math.min(100, Math.max(0, currentState.comm + delta.comm));

    const finalDim = getClippedDimensionScores(currentState);

    pathHistory.push({ x: getVisualCoord(finalDim.x), y: getVisualCoord(finalDim.y) });
    stateHistory.push({ state: JSON.parse(JSON.stringify(currentState)), x: finalDim.x, y: finalDim.y });

    if (pathHistory.length > 500) {
        pathHistory.shift();
        stateHistory.shift();
    }

    // CRITICAL: We DO NOT call updateSliderLabels or updateUI here.
    // Sliders stay at their initial config positions.
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
        const prev = stateHistory[stateHistory.length - 1];
        currentState = JSON.parse(JSON.stringify(prev.state));

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
            let color = '#d3d3d3';
            if ((i === 0 && j === 0) || (i === 0 && j === 4) || (i === 4 && j === 0) || (i === 4 && j === 4)) {
                color = '#a9a9a9';
            } else if (i >= 1 && i <= 3 && j >= 1 && j <= 3) {
                color = '#ffffff';
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
                    label: 'Trajectory',
                    data: pathHistory,
                    borderColor: 'rgba(239, 68, 68, 0.3)',
                    backgroundColor: 'rgba(239, 68, 68, 0.4)',
                    showLine: true,
                    borderWidth: 2,
                    pointRadius: 5,
                    pointHoverRadius: 7,
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
                            font: { size: 24, weight: 'bold' },
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
                            z: 100
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
    chart.data.datasets[0].data = JSON.parse(JSON.stringify(pathHistory));
    const lastPoint = pathHistory[pathHistory.length - 1];
    chart.options.plugins.annotation.annotations.currentPosMarker.xValue = lastPoint.x;
    chart.options.plugins.annotation.annotations.currentPosMarker.yValue = lastPoint.y;
    chart.update();
}

// Event Listeners
document.querySelectorAll('input[type="range"]').forEach(slider => {
    slider.addEventListener('input', (e) => {
        // While simulation is NOT running, adjusting sliders updates the "Initial State"
        if (!isRunning && stateHistory.length <= 1) {
            syncFromUI();
        } else {
            // If simulation is running or has steps, we don't let sliders affect active state
            // and we don't allow "Initial State" to be changed without a Reset.
            // (Optional: we could disable sliders or show a warning)
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
    init(); // Re-sync from sliders and clear history
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

// Boot
init();
