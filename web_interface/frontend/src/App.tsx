import React, { useState, useEffect } from 'react';
import AIInterface from './components/AIInterface';
import SimpleCameraFeed from './components/SimpleCameraFeed';
import TelemetryPage from './components/TelemetryPage';
import MiniMap from './components/MiniMap';
import DroneMap from './components/DroneMap';
import DevPage from './components/DevPage';
import RuneScapeMenu from './components/RuneScapeMenu';
import AltitudeControl from './components/AltitudeControl';
import TargetStatusDisplay from './components/TargetStatusDisplay';
import { rosbridgeClient } from './services/rosbridgeClient';
import {
  UnitSystem,
  convertDistance,
  getDistanceUnit
} from './utils/unitConversions';
import { DroneStatus } from './types/drone';
import { createDroneAPI } from './api/droneAPI';
import './App.css';

const App: React.FC = () => {
  const [currentPage, setCurrentPage] = useState<'main' | 'telemetry' | 'map' | 'ai' | 'dev'>('main');
  const [droneStatus, setDroneStatus] = useState<DroneStatus>({
    drone_name: 'drone1',
    connected: false,
    armed: false,
    flight_mode: 'UNKNOWN',
    position: { x: 0, y: 0, z: 0, yaw: 0 },
    battery: 0,
    timestamp: 0
  });

  const [isConnected, setIsConnected] = useState(false);
  const [availableDrones, setAvailableDrones] = useState<string[]>([]);
  
  // Unit preferences with localStorage persistence
  const [unitSystem, setUnitSystem] = useState<UnitSystem>(() => {
    const saved = localStorage.getItem('droneOS_units');
    return (saved as UnitSystem) || 'metric';
  });
  
  // Altitude control state for map clicks
  const [targetAltitude, setTargetAltitude] = useState(15); // Default 15m altitude
  const [maxAltitude, setMaxAltitude] = useState(50); // Default 50m max altitude
  const [droneStateSub, setDroneStateSub] = useState<string | null>(null);

  // Save unit preference to localStorage
  const changeUnitSystem = (system: UnitSystem) => {
    setUnitSystem(system);
    localStorage.setItem('droneOS_units', system);
  };

  // Initialize rosbridge connection using shared client
  useEffect(() => {
    // Set up connection status callback
    const handleConnectionStatus = (connected: boolean) => {
      setIsConnected(connected);
      if (connected) {
        // Discover available drones when connected
        discoverAvailableDrones();
      }
    };

    // Register connection callback
    rosbridgeClient.onConnectionStatusChanged(handleConnectionStatus);

    // Check current connection status and connect if needed
    const currentStatus = rosbridgeClient.getConnectionStatus();
    if (!currentStatus) {
      rosbridgeClient.connect();
    } else {
      setIsConnected(true);
      discoverAvailableDrones();
    }

    // Cleanup on unmount
    return () => {
      rosbridgeClient.removeConnectionStatusCallback(handleConnectionStatus);
    };
  }, []);
  
  // Function to refresh drone state using shared rosbridge client
  const refreshDroneState = async () => {
    try {
      const result = await rosbridgeClient.callGetStateService(droneStatus.drone_name);
      if (result.success) {
        setDroneStatus(prev => ({
          ...prev,
          connected: true,
          armed: result.arming_state === 'ARMED',
          flight_mode: result.nav_state || 'UNKNOWN',
          position: {
            x: result.local_x || 0,
            y: result.local_y || 0,
            z: result.local_z || 0,
            yaw: result.local_yaw || 0
          },
          battery: Math.round((result.battery_remaining || 0) * 100),
          timestamp: Date.now()
        }));
      }
    } catch (error) {
      console.error('[DEBUG] Refresh state error:', error);
    }
  };

  // Function to discover available drones
  const discoverAvailableDrones = async () => {
    try {
      // For now, set default drone - could be enhanced to discover dynamically via rosbridgeClient
      const discoveredDrones = ['drone1'];
      setAvailableDrones(discoveredDrones);
      
      // If current drone is not in discovered list, switch to first available
      if (discoveredDrones.length > 0 && !discoveredDrones.includes(droneStatus.drone_name)) {
        setDroneStatus(prev => ({ ...prev, drone_name: discoveredDrones[0] }));
      }
    } catch (error) {
      console.warn('Drone discovery error:', error);
      setAvailableDrones(['drone1']);
    }
  };
  
  // Start subscription to drone state using rosbridgeClient
  const startDroneStateSubscription = () => {
    // Clear any existing subscription
    if (droneStateSub) {
      rosbridgeClient.unsubscribeFromDroneState(droneStateSub);
    }
    
    // Subscribe to drone state updates
    const subscriptionId = rosbridgeClient.subscribeToDroneState(
      droneStatus.drone_name,
      (state) => {
        setDroneStatus(prev => ({
          ...prev,
          connected: true,
          armed: state.arming_state === 'ARMED',
          flight_mode: state.nav_state || 'UNKNOWN',
          position: {
            x: state.local_x || 0,
            y: state.local_y || 0,
            z: state.local_z || 0,
            yaw: state.local_yaw || 0
          },
          battery: Math.round((state.battery_remaining || 0) * 100),
          timestamp: Date.now()
        }));
      }
    );
    
    setDroneStateSub(subscriptionId);
  };
  
  // Handle drone name changes - restart subscription for new drone
  useEffect(() => {
    if (isConnected) {
      startDroneStateSubscription();
    }
  }, [isConnected, droneStatus.drone_name]);

  // Create drone API with current drone name
  const droneAPI = React.useMemo(() => createDroneAPI({
    droneName: droneStatus.drone_name,
    onRefreshState: refreshDroneState,
    onSetTargetDrone: (droneName: string) => {
      setDroneStatus(prev => ({ ...prev, drone_name: droneName }));
    }
  }), [droneStatus.drone_name]);

  return (
    <div className="app">
      <header className="app-header">
        <div className="header-nav-container">
          <h1>DroneOS Command Center</h1>
          <nav className="header-nav">
            <button
              className={`nav-btn ${currentPage === 'main' ? 'active' : ''}`}
              onClick={() => setCurrentPage('main')}
            >
              Main Dashboard
            </button>
            <button
              className={`nav-btn ${currentPage === 'telemetry' ? 'active' : ''}`}
              onClick={() => setCurrentPage('telemetry')}
            >
              Status
            </button>
            <button
              className={`nav-btn ${currentPage === 'map' ? 'active' : ''}`}
              onClick={() => setCurrentPage('map')}
            >
              Drone Map
            </button>
            <button
              className={`nav-btn ${currentPage === 'ai' ? 'active' : ''}`}
              onClick={() => setCurrentPage('ai')}
            >
              AI Assistant <span className="nav-beta-label">(BETA)</span>
            </button>
            <button
              className={`nav-btn ${currentPage === 'dev' ? 'active' : ''}`}
              onClick={() => setCurrentPage('dev')}
            >
              Dev
            </button>
          </nav>
        </div>
        <div className="connection-status">
          {/* Unit System Selector */}
          <div className="unit-selector-container">
            <label className="unit-selector-label">Units:</label>
            <select
              value={unitSystem}
              onChange={(e) => changeUnitSystem(e.target.value as UnitSystem)}
              className="unit-selector"
            >
              <option value="metric">Metric (m)</option>
              <option value="imperial">Imperial (ft)</option>
            </select>
          </div>

          <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
            {isConnected ? '🟢 Connected to rosbridge' : '🔴 Disconnected from rosbridge'}
          </span>
        </div>
      </header>

      {/* Top Status Bar - Single Row with Wrap */}
      <div className="top-status-bar">
        <div className="status-row">
          <div className="status-section">
            <span className="status-label">Last Update:</span>
            <span className="status-value">
              {droneStatus.timestamp ? new Date(droneStatus.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' }) : 'NEVER'}
            </span>
          </div>
          
          
          
          <div className="status-section">
            <span className="status-label">Connection:</span>
            <span className={`status-value ${droneStatus.connected ? 'status-connected' : 'status-disconnected'}`}>
              {droneStatus.connected ? 'ACTIVE' : 'INACTIVE'}
            </span>
          </div>
          
          <div className="status-section">
            <span className="status-label">Armed:</span>
            <span className={`status-value ${droneStatus.armed ? 'status-armed' : 'status-disarmed'}`}>
              {droneStatus.armed ? 'ARMED' : 'DISARMED'}
            </span>
          </div>
          
          <div className="status-section">
            <span className="status-label">Mode:</span>
            <span className="status-value">{droneStatus.flight_mode}</span>
          </div>
          
          <div className="status-section">
            <span className="status-label">Position:</span>
            <span className="status-value">
              ({convertDistance(droneStatus.position.x, unitSystem).toFixed(2)}, {convertDistance(droneStatus.position.y, unitSystem).toFixed(2)}, {convertDistance(Math.abs(droneStatus.position.z), unitSystem).toFixed(2)}) {getDistanceUnit(unitSystem)}
            </span>
          </div>
          
          <div className="status-section">
            <span className="status-label">Heading:</span>
            <span className="status-value">{droneStatus.position.yaw.toFixed(2)} RAD</span>
          </div>
        </div>
      </div>

      {currentPage === 'main' ? (
        <div className="app-main-new">
          {/* Main content area */}
          <div className="main-content">
            {/* Center area for camera feed only */}
            <div className="center-area">
              <SimpleCameraFeed 
                droneAPI={droneAPI}
                isConnected={isConnected}
                droneStatus={droneStatus}
                unitSystem={unitSystem}
              />
            </div>

            {/* Right panel container for both MiniMap and RuneScape menu */}
            <div className="right-panel-container">
              <TargetStatusDisplay
                droneStatus={droneStatus}
                unitSystem={unitSystem}
                convertDistance={convertDistance}
                getDistanceUnit={getDistanceUnit}
              />
              {/* MiniMap in its own container */}
              <div className="minimap-container">
                <MiniMap 
                  droneAPI={droneAPI}
                  droneStatus={droneStatus}
                  availableDrones={availableDrones}
                  unitSystem={unitSystem}
                  targetAltitude={targetAltitude}
                />
              </div>
              
              {/* Altitude Control Slider */}
              <AltitudeControl
                targetAltitude={targetAltitude}
                setTargetAltitude={setTargetAltitude}
                maxAltitude={maxAltitude}
                setMaxAltitude={setMaxAltitude}
              />
              
              {/* RuneScape-style menu */}
              <div className="right-menu">
                <RuneScapeMenu
                  droneAPI={droneAPI}
                  droneStatus={droneStatus}
                  unitSystem={unitSystem}
                  availableDrones={availableDrones}
                  isConnected={isConnected}
                />
              </div>
            </div>
          </div>


        </div>
      ) : currentPage === 'telemetry' ? (
        <main className="page-main">
          <TelemetryPage
            droneAPI={droneAPI}
            droneStatus={droneStatus}
            unitSystem={unitSystem}
          />
        </main>
      ) : currentPage === 'map' ? (
        <main className="page-main">
          <DroneMap
            droneAPI={droneAPI}
            droneStatus={droneStatus}
            availableDrones={availableDrones}
            unitSystem={unitSystem}
          />
        </main>
      ) : currentPage === 'ai' ? (
        <main className="page-main">
          <div className="ai-page-container">
            <div className="ai-page-header">
              <h2>🤖 AI Assistant</h2>
              <span className="ai-beta-badge">BETA</span>
              <div className="ai-page-description">
                Natural language drone control and mission planning
              </div>
            </div>
            <div className="ai-page-content">
              <AIInterface
                droneAPI={droneAPI}
                droneStatus={droneStatus}
              />
            </div>
          </div>
        </main>
      ) : (
        <main className="page-main">
          <DevPage
            ros={null}
            isConnected={isConnected}
          />
        </main>
      )}

      {/* Bottom Status Bar */}
      <footer className="bottom-status-bar">
        <div className="status-bar-left">
        </div>

        <div className="status-bar-right">
          <span className={`status-item battery ${
            droneStatus.battery > 50 ? 'battery-good' :
            droneStatus.battery > 25 ? 'battery-warning' : 'battery-critical'
          }`}>
            🔋 {droneStatus.battery}%
          </span>
        </div>
      </footer>
    </div>
  );
};

export default App;
