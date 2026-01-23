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
    w1: 100, w2: 50, w3: 2
};

let lr_scale = 0.25;
let history = [];

// Initialize Values
function init() {
    updateUI();
    const startX = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const startY = state.f_bal + (state.f_cha - state.f_rig) / 2;
    history = [{ x: startX, y: startY }];
    createChart();
    updateMetrics();
}

function updateUI() {
    Object.keys(state).forEach(key => {
        document.getElementById(key).value = state[key];
        document.getElementById(key + '_val').innerText = state[key];
    });
    Object.keys(weights).forEach(key => {
        document.getElementById(key).value = weights[key];
        document.getElementById(key + '_val').innerText = weights[key];
    });
    document.getElementById('lr_scale').value = lr_scale;
    document.getElementById('lr_scale_val').innerText = lr_scale;
}

function updateMetrics() {
    const B = state.c_bal + state.f_bal;
    const U = state.c_dis + state.c_enm + state.f_rig + state.f_cha;
    
    const coh_ratio = state.c_bal / ((state.c_dis + state.c_enm) / 2);
    const flex_ratio = state.f_bal / ((state.f_rig + state.f_cha) / 2);
    const total_ratio = (coh_ratio + flex_ratio) / 2;
    
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
    
    const eta = (state.comm / 100) * lr_scale;
    
    // Gradients
    const dJ_dc_bal = - (weights.w1 * U) / (2 * B * B) + weights.w3 * (x - 50);
    const dJ_df_bal = - (weights.w1 * U) / (2 * B * B) + weights.w3 * (y - 50);
    const dJ_dc_enm = (weights.w1) / (2 * B) + (weights.w3 / 2) * (x - 50);
    const dJ_dc_dis = (weights.w1) / (2 * B) - (weights.w3 / 2) * (x - 50);
    const dJ_df_cha = (weights.w1) / (2 * B) + (weights.w3 / 2) * (y - 50);
    const dJ_df_rig = (weights.w1) / (2 * B) - (weights.w3 / 2) * (y - 50);
    const dJ_dcomm = - weights.w2;

    // Updates
    state.c_bal = Math.min(100, Math.max(0, state.c_bal - eta * dJ_dc_bal));
    state.f_bal = Math.min(100, Math.max(0, state.f_bal - eta * dJ_df_bal));
    state.c_enm = Math.min(100, Math.max(0, state.c_enm - eta * dJ_dc_enm));
    state.c_dis = Math.min(100, Math.max(0, state.c_dis - eta * dJ_dc_dis));
    state.f_cha = Math.min(100, Math.max(0, state.f_cha - eta * dJ_df_cha));
    state.f_rig = Math.min(100, Math.max(0, state.f_rig - eta * dJ_df_rig));
    state.comm = Math.min(100, Math.max(0, state.comm - eta * dJ_dcomm));

    const nextX = state.c_bal + (state.c_enm - state.c_dis) / 2;
    const nextY = state.f_bal + (state.f_cha - state.f_rig) / 2;
    
    history.push({ x: nextX, y: nextY });
    if (history.length > 50) history.shift();

    updateUI();
    updateMetrics();
    updateChart();

    if (isRunning) {
        animationId = setTimeout(step, 100);
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
                    data: history,
                    borderColor: '#38bdf8',
                    showLine: true,
                    borderWidth: 2,
                    pointRadius: 0,
                    tension: 0.1
                },
                {
                    label: 'Current Position',
                    data: [history[history.length - 1]],
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
                x: { min: -50, max: 150, title: { display: true, text: 'Cohesion Offset' } },
                y: { min: -50, max: 150, title: { display: true, text: 'Flexibility Offset' } }
            },
            plugins: {
                annotation: {
                    annotations: {
                        balancedRegion: {
                            type: 'box',
                            xMin: 25, xMax: 75, yMin: 25, yMax: 75,
                            backgroundColor: 'rgba(34, 197, 94, 0.1)',
                            borderColor: 'rgba(34, 197, 94, 0.3)',
                            borderWidth: 1,
                            label: { display: true, content: 'Balanced' }
                        },
                        targetPoint: {
                            type: 'point',
                            xValue: 50, yValue: 50,
                            backgroundColor: 'rgba(255, 255, 255, 0.5)',
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
    chart.data.datasets[0].data = history;
    chart.data.datasets[1].data = [history[history.length - 1]];
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
        
        document.getElementById(id + '_val').innerText = val;
        
        if (!isRunning) {
            const x = state.c_bal + (state.c_enm - state.c_dis) / 2;
            const y = state.f_bal + (state.f_cha - state.f_rig) / 2;
            history = [{ x, y }];
            updateChart();
            updateMetrics();
        }
    });
});

document.getElementById('run-btn').addEventListener('click', () => {
    isRunning = !isRunning;
    const btn = document.getElementById('run-btn');
    btn.innerText = isRunning ? 'Pause Simulation' : 'Play Simulation';
    btn.classList.toggle('primary', !isRunning);
    
    if (isRunning) step();
});

document.getElementById('reset-btn').addEventListener('click', () => {
    isRunning = false;
    clearTimeout(animationId);
    document.getElementById('run-btn').innerText = 'Play Simulation';
    document.getElementById('run-btn').classList.add('primary');
    
    state = {
        c_bal: 20, c_dis: 80, c_enm: 10,
        f_bal: 20, f_rig: 80, f_cha: 10,
        comm: 50
    };
    lr_scale = 0.25;
    
    init();
});

// Start
init();
