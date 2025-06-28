// Additional dashboard functionality and utilities
// This file provides extended dashboard features

class DashboardExtensions {
    constructor(dashboard) {
        this.dashboard = dashboard;
        this.notifications = [];
        this.alertThresholds = {
            latencyP95: 300,
            batteryLow: 300,
            connectionTimeout: 30
        };
        
        this.initNotifications();
        this.initKeyboardShortcuts();
    }
    
    initNotifications() {
        // Check for browser notification permission
        if ("Notification" in window && Notification.permission === "default") {
            Notification.requestPermission();
        }
    }
    
    initKeyboardShortcuts() {
        document.addEventListener('keydown', (e) => {
            // Ctrl+R or F5 to refresh dashboard
            if ((e.ctrlKey && e.key === 'r') || e.key === 'F5') {
                e.preventDefault();
                this.refreshDashboard();
            }
            
            // Ctrl+D to toggle dark mode (future feature)
            if (e.ctrlKey && e.key === 'd') {
                e.preventDefault();
                this.toggleDarkMode();
            }
            
            // Escape to dismiss notifications
            if (e.key === 'Escape') {
                this.dismissAllNotifications();
            }
        });
    }
    
    refreshDashboard() {
        this.dashboard.socket.emit('request_update');
        this.showToast('Dashboard refreshed', 'info');
    }
    
    toggleDarkMode() {
        // Future implementation for dark mode
        this.showToast('Dark mode toggle - coming soon!', 'info');
    }
    
    dismissAllNotifications() {
        const notifications = document.querySelectorAll('.notification-toast');
        notifications.forEach(notification => {
            notification.remove();
        });
    }
    
    checkAlerts(data) {
        // Check latency alerts
        if (data.latency_stats && data.latency_stats.p95 > this.alertThresholds.latencyP95) {
            this.createAlert('high-latency', `High latency detected: ${data.latency_stats.p95.toFixed(1)}ms`, 'warning');
        }
        
        // Check battery alerts
        if (data.battery_summary) {
            for (const [sensorId, battery] of Object.entries(data.battery_summary)) {
                if (battery.level_mah < this.alertThresholds.batteryLow) {
                    this.createAlert(`low-battery-${sensorId}`, `Low battery on ${sensorId}: ${battery.level_mah.toFixed(0)}mAh`, 'warning');
                }
            }
        }
        
        // Check connection health
        const lastUpdate = data.timestamp;
        const now = Date.now() / 1000;
        if (now - lastUpdate > this.alertThresholds.connectionTimeout) {
            this.createAlert('connection-timeout', 'Data updates delayed - connection issues possible', 'error');
        }
    }
    
    createAlert(id, message, type) {
        // Don't create duplicate alerts
        if (this.notifications.some(n => n.id === id)) {
            return;
        }
        
        const alert = {
            id: id,
            message: message,
            type: type,
            timestamp: Date.now()
        };
        
        this.notifications.push(alert);
        this.showNotification(alert);
        
        // Auto-dismiss after 10 seconds
        setTimeout(() => {
            this.dismissAlert(id);
        }, 10000);
    }
    
    showNotification(alert) {
        // Browser notification
        if ("Notification" in window && Notification.permission === "granted") {
            new Notification("Olympus Dashboard Alert", {
                body: alert.message,
                icon: "/static/favicon.ico"
            });
        }
        
        // In-page toast notification
        this.showToast(alert.message, alert.type);
    }
    
    showToast(message, type) {
        const toast = document.createElement('div');
        toast.className = `notification-toast fixed top-4 right-4 p-4 rounded-lg shadow-lg z-50 max-w-md transition-all duration-300`;
        
        const typeStyles = {
            'info': 'bg-blue-100 border-blue-500 text-blue-700',
            'success': 'bg-green-100 border-green-500 text-green-700',
            'warning': 'bg-yellow-100 border-yellow-500 text-yellow-700',
            'error': 'bg-red-100 border-red-500 text-red-700'
        };
        
        toast.className += ` ${typeStyles[type] || typeStyles.info} border-l-4`;
        
        toast.innerHTML = `
            <div class="flex justify-between items-start">
                <div class="flex-1">
                    <p class="font-medium">${message}</p>
                </div>
                <button class="ml-4 text-current opacity-70 hover:opacity-100" onclick="this.parentElement.parentElement.remove()">
                    <i class="fas fa-times"></i>
                </button>
            </div>
        `;
        
        document.body.appendChild(toast);
        
        // Auto-remove after 5 seconds
        setTimeout(() => {
            if (toast.parentElement) {
                toast.remove();
            }
        }, 5000);
    }
    
    dismissAlert(id) {
        this.notifications = this.notifications.filter(n => n.id !== id);
    }
    
    exportData() {
        // Export current dashboard data as JSON
        const data = {
            timestamp: new Date().toISOString(),
            latency_data: this.dashboard.charts.latency.data.datasets[0].data,
            battery_data: this.dashboard.charts.battery.data,
            alerts: this.notifications
        };
        
        const blob = new Blob([JSON.stringify(data, null, 2)], {type: 'application/json'});
        const url = URL.createObjectURL(blob);
        
        const a = document.createElement('a');
        a.href = url;
        a.download = `olympus-dashboard-${Date.now()}.json`;
        a.click();
        
        URL.revokeObjectURL(url);
        this.showToast('Dashboard data exported', 'success');
    }
    
    getSystemHealth() {
        // Calculate overall system health score
        const metrics = {
            latency: 0,
            connectivity: 0,
            battery: 0,
            sensors: 0
        };
        
        // Implementation would calculate health scores based on current data
        // Return score from 0-100
        return Object.values(metrics).reduce((a, b) => a + b, 0) / Object.keys(metrics).length;
    }
}

// Utility functions for dashboard
const DashboardUtils = {
    formatLatency: (ms) => {
        if (ms < 1) return `${(ms * 1000).toFixed(0)}μs`;
        if (ms < 1000) return `${ms.toFixed(1)}ms`;
        return `${(ms / 1000).toFixed(2)}s`;
    },
    
    formatBattery: (mah) => {
        if (mah >= 1000) return `${(mah / 1000).toFixed(1)}Ah`;
        return `${mah.toFixed(0)}mAh`;
    },
    
    formatTimestamp: (timestamp) => {
        return new Date(timestamp * 1000).toLocaleString();
    },
    
    getLatencyColor: (latency) => {
        if (latency < 50) return '#10b981'; // green
        if (latency < 150) return '#f59e0b'; // yellow
        return '#ef4444'; // red
    },
    
    getBatteryColor: (percentage) => {
        if (percentage > 50) return '#10b981'; // green
        if (percentage > 20) return '#f59e0b'; // yellow
        return '#ef4444'; // red
    }
};

// Initialize extensions when dashboard is ready
document.addEventListener('DOMContentLoaded', () => {
    // Wait for main dashboard to initialize
    setTimeout(() => {
        if (window.dashboard) {
            window.dashboardExtensions = new DashboardExtensions(window.dashboard);
            
            // Hook into dashboard updates to check for alerts
            const originalUpdate = window.dashboard.updateDashboard;
            window.dashboard.updateDashboard = function(data) {
                originalUpdate.call(this, data);
                window.dashboardExtensions.checkAlerts(data);
            };
        }
    }, 1000);
});

// Export utility for console access
window.DashboardUtils = DashboardUtils;