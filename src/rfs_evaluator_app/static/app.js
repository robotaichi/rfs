/**
 * FACES-IV Validation App — Frontend Logic
 */
(function () {
    'use strict';

    // ── State ─────────────────────────────────────────────────────────────
    let archiveName = '';
    let archiveData = null;    // { sessions, conversations, evaluations, items, subscales }
    let currentSessionIdx = 0; // index into archiveData.sessions
    let evaluatorScores = {};  // { session_id: { item_num_str: score } }
    let evaluatorName = '';

    // ── DOM Refs ──────────────────────────────────────────────────────────
    const $archiveScreen = document.getElementById('screen-archive');
    const $evalScreen = document.getElementById('screen-evaluation');
    const $resultScreen = document.getElementById('screen-results');
    const $archiveList = document.getElementById('archive-list');
    const $headerBadge = document.getElementById('header-badge');

    // ── Screen Management ─────────────────────────────────────────────────
    function showScreen(screen) {
        document.querySelectorAll('.screen').forEach(s => s.classList.remove('active'));
        screen.classList.add('active');
    }

    // ── Init ──────────────────────────────────────────────────────────────
    async function init() {
        showScreen($archiveScreen);
        $archiveList.innerHTML = '<div class="loading"><div class="spinner"></div>Loading archives...</div>';

        try {
            const res = await fetch('/api/archives');
            const data = await res.json();

            if (!data.archives || data.archives.length === 0) {
                $archiveList.innerHTML = '<div class="empty-state"><div class="icon">📁</div><p>No archives found.</p></div>';
                return;
            }

            $archiveList.innerHTML = data.archives.map(a => `
        <div class="archive-item" data-name="${a.name}">
          <div>
            <div class="name">📂 ${a.name}</div>
            <div class="date">${a.display}</div>
          </div>
          <div class="arrow">→</div>
        </div>
      `).join('');

            $archiveList.querySelectorAll('.archive-item').forEach(el => {
                el.addEventListener('click', () => loadArchive(el.dataset.name));
            });
        } catch (e) {
            $archiveList.innerHTML = `<div class="empty-state"><p>Error loading archives: ${e.message}</p></div>`;
        }
    }

    // ── Load Archive ──────────────────────────────────────────────────────
    async function loadArchive(name) {
        archiveName = name;
        $headerBadge.textContent = name;

        $archiveList.innerHTML = '<div class="loading"><div class="spinner"></div>Loading archive data...</div>';

        try {
            const res = await fetch(`/api/archive/${name}`);
            archiveData = await res.json();

            if (archiveData.error) {
                alert('Error: ' + archiveData.error);
                return;
            }

            // Filter sessions to only those that have evaluation data
            const evalSessions = Object.keys(archiveData.evaluations);
            archiveData.sessions = archiveData.sessions.filter(s => evalSessions.includes(s));

            if (archiveData.sessions.length === 0) {
                alert('No evaluation data found in this archive.');
                init();
                return;
            }

            // Initialize evaluator scores
            evaluatorScores = {};
            archiveData.sessions.forEach(s => { evaluatorScores[s] = {}; });

            currentSessionIdx = 0;
            renderEvaluation();
        } catch (e) {
            alert('Error loading archive: ' + e.message);
            init();
        }
    }

    // ── Render Evaluation Screen ──────────────────────────────────────────
    function renderEvaluation() {
        const session = archiveData.sessions[currentSessionIdx];
        // For the conversation, we need the session BEFORE (since eval happens after conversation)
        // e.g., S1 eval is about the conversation in S0
        const convSessionIdx = parseInt(session.substring(1)) - 1;
        const convSessionId = `S${convSessionIdx}`;
        const conversations = archiveData.conversations[convSessionId] || [];
        const totalSessions = archiveData.sessions.length;
        const progress = ((currentSessionIdx + 1) / totalSessions) * 100;

        const evalContainer = document.getElementById('eval-container');
        evalContainer.innerHTML = `
      <button class="back-btn" id="btn-back-archive">← Back to Archives</button>

      <!-- Session Progress -->
      <div class="session-progress">
        <span class="label">Session ${currentSessionIdx + 1} / ${totalSessions}</span>
        <div class="progress-bar-container">
          <div class="progress-bar-fill" style="width: ${progress}%"></div>
        </div>
        <span class="count">${session} (Conversation: ${convSessionId})</span>
      </div>

      <!-- Evaluator Name -->
      <div class="evaluator-input-group">
        <label for="evaluator-name">👤 Evaluator Name:</label>
        <input type="text" id="evaluator-name" placeholder="Your name" value="${evaluatorName}">
      </div>

      <!-- Conversation History -->
      <div class="card">
        <div class="card-title"><span class="icon">💬</span> Conversation History (${convSessionId})</div>
        <div class="conversation-area" id="conv-area">
          ${conversations.length === 0
                ? '<div class="empty-state"><p>No conversation data for this session.</p></div>'
                : conversations.map(c => renderBubble(c)).join('')
            }
        </div>
      </div>

      <!-- FACES-IV Questionnaire -->
      <div class="card" style="margin-top: 20px;">
        <div class="card-title"><span class="icon">📋</span> FACES-IV Questionnaire</div>
        <div class="faces-description">
          <strong>Instructions:</strong> Based on the conversation above, rate how you perceive the
          family's state for each of the following 62 items on a 5-point scale.<br>
          <strong>1</strong> = Strongly Disagree &nbsp;
          <strong>2</strong> = Generally Disagree &nbsp;
          <strong>3</strong> = Undecided &nbsp;
          <strong>4</strong> = Generally Agree &nbsp;
          <strong>5</strong> = Strongly Agree
        </div>

        <!-- Scale Header -->
        <div class="scale-header">
          <span>#</span>
          <span>Item</span>
          <span>1</span><span>2</span><span>3</span><span>4</span><span>5</span>
        </div>

        <!-- Items -->
        <div class="questionnaire-items" id="q-items">
          ${renderItems(session)}
        </div>
      </div>

      <!-- Action Buttons -->
      <div class="btn-group" style="justify-content: flex-end;">
        ${currentSessionIdx > 0
                ? '<button class="btn btn-secondary" id="btn-prev-session">← Previous Session</button>'
                : ''
            }
        ${currentSessionIdx < totalSessions - 1
                ? '<button class="btn btn-primary" id="btn-next-session">Next Session →</button>'
                : '<button class="btn btn-success" id="btn-view-results">📊 View Results</button>'
            }
      </div>
    `;

        showScreen($evalScreen);

        // Wire up events
        document.getElementById('btn-back-archive').addEventListener('click', () => {
            if (confirm('Go back to archive selection? Your progress will be lost.')) init();
        });

        const nameInput = document.getElementById('evaluator-name');
        nameInput.addEventListener('input', () => { evaluatorName = nameInput.value; });

        const btnPrev = document.getElementById('btn-prev-session');
        if (btnPrev) btnPrev.addEventListener('click', () => { saveCurrentScores(); currentSessionIdx--; renderEvaluation(); });

        const btnNext = document.getElementById('btn-next-session');
        if (btnNext) btnNext.addEventListener('click', () => {
            saveCurrentScores();
            if (!validateCurrentSession()) return;
            currentSessionIdx++;
            renderEvaluation();
        });

        const btnResults = document.getElementById('btn-view-results');
        if (btnResults) btnResults.addEventListener('click', () => {
            saveCurrentScores();
            if (!validateCurrentSession()) return;
            renderResults();
        });

        // Wire up radio buttons
        document.querySelectorAll('.q-radio').forEach(radio => {
            radio.addEventListener('change', (e) => {
                const item = e.target.closest('.q-item');
                item.classList.add('answered');
                item.classList.remove('unanswered-highlight');
            });
        });

        // Restore saved scores for this session
        restoreCurrentScores();

        // Scroll conversation to bottom
        const convArea = document.getElementById('conv-area');
        if (convArea) convArea.scrollTop = convArea.scrollHeight;
    }

    function renderBubble(conv) {
        if (conv.raw) return `<div class="chat-bubble">${conv.raw}</div>`;
        const role = conv.speaker || 'unknown';
        return `
      <div class="chat-bubble ${role}">
        <div class="speaker-label">${role} → ${conv.target || ''}</div>
        ${conv.text || ''}
      </div>
    `;
    }

    function renderItems(session) {
        const items = archiveData.items;
        let html = '';
        let prevSubscale = '';

        for (const item of items) {
            if (item.subscale !== prevSubscale) {
                html += `<div class="subscale-divider">${item.subscale} (Items ${getSubscaleRange(item.subscale)})</div>`;
                prevSubscale = item.subscale;
            }

            const savedScore = evaluatorScores[session]?.[String(item.num)] || null;

            html += `
        <div class="q-item ${savedScore ? 'answered' : ''}" data-item="${item.num}">
          <span class="q-num">${item.num}</span>
          <span class="q-text">${item.text}</span>
          ${[1, 2, 3, 4, 5].map(v => `
            <div class="radio-cell">
              <label class="radio-btn">
                <input type="radio" class="q-radio" name="q_${item.num}" value="${v}"
                  ${savedScore == v ? 'checked' : ''}>
                <span class="radio-circle"><span class="radio-dot"></span></span>
              </label>
            </div>
          `).join('')}
        </div>
      `;
        }
        return html;
    }

    function getSubscaleRange(name) {
        const items = archiveData.subscales[name];
        if (!items || items.length === 0) return '';
        return `${items[0]}-${items[items.length - 1]}`;
    }

    function saveCurrentScores() {
        const session = archiveData.sessions[currentSessionIdx];
        if (!evaluatorScores[session]) evaluatorScores[session] = {};
        document.querySelectorAll('.q-radio:checked').forEach(radio => {
            const itemNum = radio.name.replace('q_', '');
            evaluatorScores[session][itemNum] = parseInt(radio.value);
        });
    }

    function restoreCurrentScores() {
        const session = archiveData.sessions[currentSessionIdx];
        const saved = evaluatorScores[session] || {};
        for (const [itemNum, score] of Object.entries(saved)) {
            const radio = document.querySelector(`input[name="q_${itemNum}"][value="${score}"]`);
            if (radio) {
                radio.checked = true;
                const item = radio.closest('.q-item');
                if (item) item.classList.add('answered');
            }
        }
    }

    function validateCurrentSession() {
        const session = archiveData.sessions[currentSessionIdx];
        const answered = Object.keys(evaluatorScores[session] || {}).length;
        if (answered < 62) {
            const unanswered = 62 - answered;
            const proceed = confirm(`You have ${unanswered} unanswered item(s). Continue anyway?`);
            if (!proceed) {
                // Highlight unanswered items
                document.querySelectorAll('.q-item').forEach(item => {
                    if (!item.classList.contains('answered')) {
                        item.classList.add('unanswered-highlight');
                    }
                });
                return false;
            }
        }
        return true;
    }

    // ── Render Results Screen ─────────────────────────────────────────────
    let resultSessionIdx = 0;

    function renderResults() {
        resultSessionIdx = 0;
        showScreen($resultScreen);
        renderResultsForSession();
    }

    function renderResultsForSession() {
        const session = archiveData.sessions[resultSessionIdx];
        const evalData = archiveData.evaluations[session] || { members: {}, mean: {} };
        const memberNames = Object.keys(evalData.members).sort();
        const humanScores = evaluatorScores[session] || {};

        const resultContainer = document.getElementById('result-container');
        resultContainer.innerHTML = `
      <button class="back-btn" id="btn-back-eval">← Back to Evaluation</button>

      <!-- Session Tabs -->
      <div class="session-tabs" id="session-tabs">
        ${archiveData.sessions.map((s, i) => `
          <button class="session-tab ${i === resultSessionIdx ? 'active' : ''}" data-idx="${i}">
            ${s}
          </button>
        `).join('')}
      </div>

      <!-- Results Table -->
      <div class="card">
        <div class="card-title"><span class="icon">📊</span> Score Comparison — ${session}</div>
        <div class="results-table-container">
          <table class="results-table">
            <thead>
              <tr>
                <th>#</th>
                <th>Item</th>
                ${memberNames.map(m => `<th class="col-${m}">${m}</th>`).join('')}
                <th class="col-mean">Mean</th>
                <th class="col-evaluator">Evaluator</th>
                <th>Diff</th>
              </tr>
            </thead>
            <tbody>
              ${renderResultRows(session, evalData, memberNames, humanScores)}
            </tbody>
          </table>
        </div>
      </div>

      <!-- Save Actions -->
      <div class="btn-group">
        <button class="btn btn-success" id="btn-save-csv">💾 Save as CSV</button>
      </div>
      <div class="save-result" id="save-result">
        <strong>✅ Saved successfully!</strong>
        <div class="filepath" id="save-filepath"></div>
        <button class="btn btn-secondary" id="btn-open-explorer" style="margin-top: 10px;">
          📂 Open in File Explorer
        </button>
      </div>
    `;

        // Wire up events
        document.getElementById('btn-back-eval').addEventListener('click', () => {
            currentSessionIdx = archiveData.sessions.length - 1;
            renderEvaluation();
        });

        document.querySelectorAll('.session-tab').forEach(tab => {
            tab.addEventListener('click', () => {
                resultSessionIdx = parseInt(tab.dataset.idx);
                renderResultsForSession();
            });
        });

        document.getElementById('btn-save-csv').addEventListener('click', saveCSV);
    }

    function renderResultRows(session, evalData, memberNames, humanScores) {
        let html = '';
        let prevSubscale = '';

        for (const item of archiveData.items) {
            if (item.subscale !== prevSubscale) {
                const colspan = memberNames.length + 5;
                html += `<tr class="subscale-row"><td colspan="${colspan}">${item.subscale}</td></tr>`;
                prevSubscale = item.subscale;
            }

            const itemKey = String(item.num);
            const meanScore = evalData.mean[itemKey] ?? '';
            const humanScore = humanScores[itemKey] ?? '';
            const diff = (humanScore !== '' && meanScore !== '')
                ? (humanScore - parseFloat(meanScore)).toFixed(1)
                : '';
            const absDiff = Math.abs(parseFloat(diff) || 0);
            const diffClass = diff === '' ? '' :
                absDiff === 0 ? 'diff-exact' :
                    absDiff <= 1 ? 'diff-close' : 'diff-far';

            html += `
        <tr>
          <td>${item.num}</td>
          <td>${item.text}</td>
          ${memberNames.map(m => {
                const s = evalData.members[m]?.[itemKey] ?? '';
                return `<td class="col-${m}">${s}</td>`;
            }).join('')}
          <td class="col-mean">${typeof meanScore === 'number' ? meanScore.toFixed(1) : meanScore}</td>
          <td class="col-evaluator"><strong>${humanScore}</strong></td>
          <td class="${diffClass}">${diff}</td>
        </tr>
      `;
        }
        return html;
    }

    // ── Save CSV ──────────────────────────────────────────────────────────
    async function saveCSV() {
        const name = evaluatorName.trim() || 'anonymous';
        try {
            const res = await fetch('/api/save_csv', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    archive_name: archiveName,
                    evaluator_name: name,
                    results: evaluatorScores,
                }),
            });
            const data = await res.json();
            if (data.error) {
                alert('Error: ' + data.error);
                return;
            }

            const saveResult = document.getElementById('save-result');
            const saveFilepath = document.getElementById('save-filepath');
            saveResult.classList.add('visible');
            saveFilepath.textContent = data.saved_files.join('\n');

            // Wire up open explorer button
            const btnOpen = document.getElementById('btn-open-explorer');
            btnOpen.addEventListener('click', async () => {
                const firstFile = data.saved_files[0];
                try {
                    await fetch('/api/open_file', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ filepath: firstFile }),
                    });
                } catch (e) {
                    alert('Could not open file explorer: ' + e.message);
                }
            });
        } catch (e) {
            alert('Error saving: ' + e.message);
        }
    }

    // ── Start ─────────────────────────────────────────────────────────────
    init();
})();
