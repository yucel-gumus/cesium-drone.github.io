class DroneUIController {
    constructor() {
        this.connectionStatus = document.getElementById('connectionStatus');
        this.batteryStatus = document.getElementById('batteryStatus');
        this.altitudeStatus = document.getElementById('altitudeStatus');
        this.speedStatus = document.getElementById('speedStatus');
        this.droneStatus = document.getElementById('droneStatus');
        this.startTime = null;
        this.durationInterval = null;
        this.initializeControls();
    }
    initializeControls() {
        this.updateConnectionStatus('Bağlı');
        this.updateBatteryStatus(100);

    }
    updateConnectionStatus(status) {
        this.connectionStatus.textContent = status;
        this.connectionStatus.className = 'status-value ' + (status === 'Bağlı' ? 'connected' : 'disconnected');
    }
    updateBatteryStatus(percentage) {
        this.batteryStatus.textContent = `${percentage}%`;
        if (percentage <= 20) {
            this.batteryStatus.style.color = '#e74c3c';
        }
    }
    updateAltitude(height) {
        this.altitudeStatus.textContent = `${height.toFixed(1)}m`;
    }
    updateSpeed(speed) {
        this.speedStatus.textContent = `${speed.toFixed(1)} km/s`;
    }
    updateFromPosition(position) {
        if (!position) return;

        const cartographic = Cesium.Cartographic.fromCartesian(position);
        const height = cartographic.height; 
        this.updateAltitude(height);
        const speed = Math.random() * 5 + 15; 
        this.updateSpeed(speed);
    }
}
window.droneUI = new DroneUIController();
