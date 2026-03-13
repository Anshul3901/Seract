// Common utilities

function formatTimestamp(timestamp) {
    if (!timestamp) return '-';
    const date = new Date(timestamp);
    return date.toLocaleString();
}

function getDecisionBadgeClass(decision) {
    if (decision && decision.toLowerCase().includes('restock')) {
        return 'restock';
    } else if (decision && decision.toLowerCase().includes('recycle')) {
        return 'recycle';
    }
    return '';
}

function getImageUrl(imagePath) {
    if (!imagePath) return '';
    // Convert absolute path to API endpoint
    return `/api/image/${imagePath}`;
}

