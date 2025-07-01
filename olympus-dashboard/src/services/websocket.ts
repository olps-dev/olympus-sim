import { io, Socket } from 'socket.io-client';

class WebSocketService {
  private socket: Socket | null = null;
  private url: string = 'http://localhost:3001';

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.socket = io(this.url);

      this.socket.on('connect', () => {
        console.log('Connected to backend');
        resolve();
      });

      this.socket.on('connect_error', (error) => {
        console.warn('Failed to connect to backend:', error.message);
        reject(error);
      });

      this.socket.on('sensor_data', (data) => {
        // Handle incoming sensor data
        console.log('Received sensor data:', data);
      });

      this.socket.on('automation_event', (event) => {
        // Handle automation events
        console.log('Automation event:', event);
      });
    });
  }

  disconnect(): void {
    if (this.socket) {
      this.socket.disconnect();
      this.socket = null;
    }
  }

  emit(event: string, data: any): void {
    if (this.socket) {
      this.socket.emit(event, data);
    }
  }

  on(event: string, callback: (data: any) => void): void {
    if (this.socket) {
      this.socket.on(event, callback);
    }
  }

  // Script management methods
  saveNodeScript(nodeId: string, script: string): void {
    this.emit('save_node_script', { nodeId, script });
  }

  executeNodeScript(nodeId: string): void {
    this.emit('execute_node_script', { nodeId });
  }

  stopNodeScript(nodeId: string): void {
    this.emit('stop_node_script', { nodeId });
  }
}

export const websocketService = new WebSocketService();