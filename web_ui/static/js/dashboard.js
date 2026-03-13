// Dashboard JavaScript

let socket = null;
let currentState = {
    latest_camera_frame: null,
    captured_images: {},
    processing_status: 'idle',
    current_view: null,
    latest_decision: null
};

// Initialize WebSocket connection
function initSocket() {
    socket = io();
    
    socket.on('connect', () => {
        console.log('Connected to server');
    });
    
    socket.on('state_update', (state) => {
        currentState = state;
        updateUI(state);
    });
    
    socket.on('disconnect', () => {
        console.log('Disconnected from server');
    });
}

// Update UI based on state
function updateUI(state) {
    // Update camera feed
    if (state.latest_camera_frame) {
        const cameraImg = document.getElementById('camera-feed');
        const placeholder = document.getElementById('camera-placeholder');
        if (cameraImg && placeholder) {
            try {
                cameraImg.src = 'data:image/jpeg;base64,' + state.latest_camera_frame;
                cameraImg.style.display = 'block';
                placeholder.style.display = 'none';
                cameraImg.onerror = function() {
                    console.error('Failed to load camera image');
                    cameraImg.style.display = 'none';
                    placeholder.style.display = 'flex';
                };
            } catch (e) {
                console.error('Error setting camera image:', e);
            }
        }
    } else {
        // Show placeholder if no camera frame
        const cameraImg = document.getElementById('camera-feed');
        const placeholder = document.getElementById('camera-placeholder');
        if (cameraImg && placeholder) {
            cameraImg.style.display = 'none';
            placeholder.style.display = 'flex';
        }
    }
    
    // Update status bar
    updateStatusBar(state);
    
    // Update system stage
    updateSystemStage(state.system_stage || state.processing_status || 'idle', state.latest_decision_text);
    
    // Update decision results (if available)
    if (state.latest_decision) {
        updateDecisionResults(state.latest_decision, state.latest_summary, state.latest_decision_text);
    }
}

function updateStatusBar(state) {
    const statusEl = document.getElementById('processing-status');
    const viewEl = document.getElementById('current-view');
    const decisionEl = document.getElementById('decision-text');
    
    if (statusEl) {
        statusEl.textContent = state.processing_status || 'Idle';
        statusEl.className = 'status-value ' + getDecisionBadgeClass(state.processing_status);
    }
    
    if (viewEl) {
        viewEl.textContent = state.current_view || '-';
    }
    
    if (decisionEl) {
        decisionEl.textContent = state.latest_decision_text || '-';
        decisionEl.className = 'status-value ' + getDecisionBadgeClass(state.latest_decision_text);
    }
}

function updateSystemStage(stage, decisionText) {
    // Hide all stages
    const stages = ['idle', 'scanning', 'analyzing', 'accept', 'reject'];
    stages.forEach(s => {
        const el = document.getElementById(`stage-${s}`);
        if (el) el.style.display = 'none';
    });
    
    // Show appropriate stage
    let stageToShow = stage;
    
    // Map processing_status to stage if system_stage not available
    if (!stageToShow || stageToShow === 'idle') {
        stageToShow = 'idle';
    } else if (stageToShow === 'scanning' || stageToShow === 'capturing') {
        stageToShow = 'scanning';
    } else if (stageToShow === 'analyzing' || stageToShow === 'processing') {
        stageToShow = 'analyzing';
    } else if (stageToShow === 'accept' || (decisionText && (decisionText.toLowerCase().includes('restock') || decisionText.toLowerCase().includes('resell')))) {
        stageToShow = 'accept';
    } else if (stageToShow === 'reject' || (decisionText && (decisionText.toLowerCase().includes('recycle') || decisionText.toLowerCase().includes('review')))) {
        stageToShow = 'reject';
    } else if (stageToShow === 'complete') {
        // If complete but no decision text, keep analyzing
        if (decisionText && decisionText !== '-') {
            if (decisionText.toLowerCase().includes('restock') || decisionText.toLowerCase().includes('resell')) {
                stageToShow = 'accept';
            } else if (decisionText.toLowerCase().includes('recycle') || decisionText.toLowerCase().includes('review')) {
                stageToShow = 'reject';
            } else {
                stageToShow = 'analyzing';
            }
        } else {
            stageToShow = 'analyzing';
        }
    }
    
    const stageEl = document.getElementById(`stage-${stageToShow}`);
    if (stageEl) {
        stageEl.style.display = 'block';
    }
}

function updateDecisionResults(decisionData, summary, decisionText) {
    const panel = document.getElementById('decision-panel');
    if (!panel) return;
    
    panel.style.display = 'block';
    
    const titleEl = document.getElementById('decision-title');
    const badgeEl = document.getElementById('decision-badge');
    const summaryEl = document.getElementById('decision-summary');
    const detailsEl = document.getElementById('decision-details');
    
    if (titleEl) {
        titleEl.textContent = 'Inspection Decision';
    }
    
    if (badgeEl) {
        badgeEl.textContent = decisionText || 'Unknown';
        badgeEl.className = 'decision-badge ' + getDecisionBadgeClass(decisionText);
    }
    
    if (summaryEl) {
        summaryEl.innerHTML = `<p>${summary || 'No summary available'}</p>`;
    }
    
    if (detailsEl && decisionData.view_results) {
        const views = Object.keys(decisionData.view_results);
        detailsEl.innerHTML = views.map(viewName => {
            const viewData = decisionData.view_results[viewName];
            return `
                <div class="view-result">
                    <h4>${viewName.charAt(0).toUpperCase() + viewName.slice(1)} View</h4>
                    <p><strong>Decision:</strong> ${viewData.decision || 'N/A'}</p>
                    <p><strong>Defects:</strong> ${viewData.defect_count || 0}</p>
                    ${viewData.openai_metadata && viewData.openai_metadata.summary ? 
                        `<p><strong>Summary:</strong> ${viewData.openai_metadata.summary}</p>` : ''}
                </div>
            `;
        }).join('');
    }
}

// Fetch initial state
async function fetchInitialState() {
    try {
        const response = await fetch('/api/state');
        const state = await response.json();
        updateUI(state);
    } catch (error) {
        console.error('Error fetching initial state:', error);
    }
}

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    initSocket();
    fetchInitialState();
    
    // Poll for updates as fallback
    setInterval(fetchInitialState, 2000);
});

