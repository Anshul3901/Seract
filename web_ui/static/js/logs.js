// Logs page JavaScript

let pastItems = [];

// Load past items
async function loadPastItems() {
    const loadingEl = document.getElementById('logs-loading');
    const contentEl = document.getElementById('logs-content');
    const gridEl = document.getElementById('logs-grid');
    
    try {
        const response = await fetch('/api/past_items');
        pastItems = await response.json();
        
        if (pastItems.length === 0) {
            gridEl.innerHTML = '<p class="empty-message">No past items found</p>';
        } else {
            gridEl.innerHTML = pastItems.map(item => {
                const badgeClass = getDecisionBadgeClass(item.decision);
                return `
                    <div class="log-item" onclick="showItemDetails('${item.item_name}')">
                        <div class="log-item-header">
                            <div class="log-item-title">${item.item_name}</div>
                            <span class="log-item-badge ${badgeClass}">${item.decision || 'Unknown'}</span>
                        </div>
                        <div class="log-item-info">
                            <strong>Object Type:</strong> ${item.object_type || 'Unknown'}
                        </div>
                        <div class="log-item-info">
                            <strong>Images:</strong> ${item.image_count || 0}
                        </div>
                        ${item.timestamp ? `
                            <div class="log-item-info">
                                <strong>Date:</strong> ${formatTimestamp(item.timestamp)}
                            </div>
                        ` : ''}
                        ${item.summary ? `
                            <div class="log-item-summary">${item.summary}</div>
                        ` : ''}
                    </div>
                `;
            }).join('');
        }
        
        // Remove any back buttons from log items
        const logItems = gridEl.querySelectorAll('.log-item');
        logItems.forEach(item => {
            const backButtons = item.querySelectorAll('button');
            backButtons.forEach(btn => {
                const text = btn.textContent.toLowerCase().trim();
                const className = btn.className.toLowerCase();
                if (text.includes('back') || className.includes('back')) {
                    btn.remove();
                }
            });
        });
        
        loadingEl.style.display = 'none';
        contentEl.style.display = 'block';
    } catch (error) {
        console.error('Error loading past items:', error);
        loadingEl.innerHTML = '<p class="empty-message">Error loading past items</p>';
    }
}

// Show item details modal
async function showItemDetails(itemName) {
    const modal = document.getElementById('item-modal');
    const modalBody = document.getElementById('modal-body');
    
    try {
        const response = await fetch(`/api/past_item/${itemName}`);
        const itemData = await response.json();
        
        if (itemData.error) {
            alert('Error loading item: ' + itemData.error);
            return;
        }
        
        const result = itemData.result || {};
        const images = itemData.images || [];
        const badgeClass = getDecisionBadgeClass(result.result);
        
        modalBody.innerHTML = `
            <h2>${itemName}</h2>
            
            <div style="margin: 1.5rem 0;">
                <div style="display: flex; gap: 1rem; align-items: center; margin-bottom: 1rem;">
                    <span class="decision-badge ${badgeClass}">${result.result || 'Unknown'}</span>
                    ${result.object_identification && result.object_identification.object_type ? 
                        `<span><strong>Object Type:</strong> ${result.object_identification.object_type}</span>` : ''}
                </div>
                
                ${result.combined_summary || result.explanation ? `
                    <div class="decision-summary">
                        <h3>Summary</h3>
                        <p>${result.combined_summary || result.explanation}</p>
                    </div>
                ` : ''}
                
                ${result.total_defects !== undefined ? `
                    <div style="margin: 1rem 0;">
                        <strong>Total Defects:</strong> ${result.total_defects}
                    </div>
                ` : ''}
            </div>
            
            ${images.length > 0 ? `
                <h3>Images</h3>
                <div class="modal-images">
                    ${images.map(img => `
                        <div class="modal-image-item">
                            <img src="${getImageUrl(img.path)}" alt="${img.view}" 
                                 onerror="this.src='data:image/svg+xml,%3Csvg xmlns=\\'http://www.w3.org/2000/svg\\' width=\\'200\\' height=\\'200\\'%3E%3Crect fill=\\'%23ddd\\' width=\\'200\\' height=\\'200\\'/%3E%3Ctext x=\\'50%25\\' y=\\'50%25\\' text-anchor=\\'middle\\' dy=\\'.3em\\' fill=\\'%23999\\'%3EImage%3C/text%3E%3C/svg%3E'">
                            <div class="captured-image-label">${img.view}</div>
                        </div>
                    `).join('')}
                </div>
            ` : ''}
            
            ${result.view_results ? `
                <h3 style="margin-top: 2rem;">View Details</h3>
                <div class="decision-details">
                    ${Object.keys(result.view_results).map(viewName => {
                        const viewData = result.view_results[viewName];
                        return `
                            <div class="view-result">
                                <h4>${viewName.charAt(0).toUpperCase() + viewName.slice(1)} View</h4>
                                <p><strong>Decision:</strong> ${viewData.decision || 'N/A'}</p>
                                <p><strong>Defects:</strong> ${viewData.defect_count || 0}</p>
                                ${viewData.openai_metadata && viewData.openai_metadata.summary ? 
                                    `<p><strong>Summary:</strong> ${viewData.openai_metadata.summary}</p>` : ''}
                            </div>
                        `;
                    }).join('')}
                </div>
            ` : ''}
        `;
        
        // Remove any back buttons that might exist
        // Wait for DOM to update, then remove back buttons
        setTimeout(() => {
            const allButtons = modalBody.querySelectorAll('button');
            allButtons.forEach(btn => {
                const text = btn.textContent.toLowerCase().trim();
                const className = btn.className.toLowerCase();
                if (text.includes('back') || className.includes('back')) {
                    btn.remove();
                }
            });
        }, 0);
        
        modal.style.display = 'block';
    } catch (error) {
        console.error('Error loading item details:', error);
        alert('Error loading item details');
    }
}

// Close modal
function closeModal() {
    const modal = document.getElementById('item-modal');
    modal.style.display = 'none';
}

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    loadPastItems();
    
    // Close modal on X click
    const closeBtn = document.querySelector('.modal-close');
    if (closeBtn) {
        closeBtn.addEventListener('click', closeModal);
    }
    
    // Close modal on outside click
    const modal = document.getElementById('item-modal');
    if (modal) {
        modal.addEventListener('click', (e) => {
            if (e.target === modal) {
                closeModal();
            }
        });
    }
});

