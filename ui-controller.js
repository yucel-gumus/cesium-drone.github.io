class DroneUIController {
    constructor() {
        this.connectionStatus = document.getElementById('connectionStatus');
        this.batteryStatus = document.getElementById('batteryStatus');
        this.altitudeStatus = document.getElementById('altitudeStatus');
        this.speedStatus = document.getElementById('speedStatus');

        this.startTime = null;
        this.durationInterval = null;
        this.batteryInterval = null;
        this.cesiumTickListener = null;

        this.initializeControls();
    }

    updateConnectionStatus(status) {
        if (this.connectionStatus) {
            this.connectionStatus.textContent = status;
        }
    }

    updateBatteryStatus(percentage) {
        if (this.batteryStatus) {
            this.batteryStatus.textContent = `${percentage}%`;
        }
    }

    updateAltitude(height) {
        if (this.altitudeStatus) {
            this.altitudeStatus.textContent = `${Math.round(height)}m`;
        }
    }

    updateSpeed(speed) {
        if (this.speedStatus) {
            this.speedStatus.textContent = `${Math.round(speed)} km/h`;
        }
    }

    initializeControls() {
        if (this.connectionStatus) this.updateConnectionStatus('Çevrim Dışı');
        if (this.batteryStatus) this.updateBatteryStatus(100);
        if (this.altitudeStatus) this.updateAltitude(0);
        if (this.speedStatus) this.updateSpeed(0);
    }

    stopUpdates() {
        
        if (this.batteryInterval) {
            clearInterval(this.batteryInterval);
            this.batteryInterval = null;
        }
        if (this.cesiumTickListener && window.viewer && window.viewer.clock) {
            window.viewer.clock.onTick.removeEventListener(this.cesiumTickListener);
            this.cesiumTickListener = null;
        }
        if (this.durationInterval) {
            clearInterval(this.durationInterval);
            this.durationInterval = null;
        }
    }

    startUpdates() {
        this.stopUpdates();

        let currentBattery = 100;
        this.updateBatteryStatus(currentBattery);
        this.batteryInterval = setInterval(() => {
            currentBattery = Math.max(0, currentBattery - 0.1);
            this.updateBatteryStatus(Math.round(currentBattery));
            if (currentBattery === 0) {
                console.log("Batarya bitti!");
                this.updateConnectionStatus('Batarya Bitti!');
                this.stopUpdates();
            }
        }, 1000);

        this.cesiumTickListener = (clock) => {
            const entity = window.viewer.trackedEntity;
            if (entity && entity.position) {
                const currentTime = clock.currentTime;
                const position = entity.position.getValue(currentTime);

                if (position) {
                    const cartographic = Cesium.Cartographic.fromCartesian(position);
                    const height = cartographic.height;
                    this.updateAltitude(height-1150);

                    let speed = 0;
                    try {
                        if (entity.velocity) {
                            const velocityVector = entity.velocity.getValue(currentTime);
                            if (velocityVector) {
                                speed = Cesium.Cartesian3.magnitude(velocityVector);
                            }
                        }
                    } catch (e) {
                        console.warn('Hız hesaplanırken hata:', e);
                    }
                    this.updateSpeed(speed * 3.6);
                }
            }
        };

        if (window.viewer && window.viewer.clock) {
            window.viewer.clock.onTick.addEventListener(this.cesiumTickListener);
        } else {
            console.error('Viewer veya clock bulunamadı!');
        }
    }
}

export default DroneUIController;