// API service for communicating with Olympus backend

const BASE_URL = 'http://localhost:3001';

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
}

class OlympusAPI {
  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<ApiResponse<T>> {
    try {
      const response = await fetch(`${BASE_URL}${endpoint}`, {
        headers: {
          'Content-Type': 'application/json',
          ...options.headers,
        },
        ...options,
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return { success: true, data };
    } catch (error) {
      console.error(`API request failed for ${endpoint}:`, error);
      return { 
        success: false, 
        error: error instanceof Error ? error.message : 'Unknown error' 
      };
    }
  }

  async getSimulationState() {
    return this.request('/simulation_state');
  }

  async createNode(config: any) {
    return this.request('/create_node', {
      method: 'POST',
      body: JSON.stringify(config),
    });
  }

  async createSensor(config: any) {
    return this.request('/create_sensor', {
      method: 'POST',
      body: JSON.stringify(config),
    });
  }

  async createRoom(config: any) {
    return this.request('/create_room', {
      method: 'POST',
      body: JSON.stringify(config),
    });
  }

  async startSimulation() {
    return this.request('/start_simulation', {
      method: 'POST',
      body: JSON.stringify({}),
    });
  }

  async stopSimulation() {
    return this.request('/stop_simulation', {
      method: 'POST',
      body: JSON.stringify({}),
    });
  }

  async checkHealth() {
    return this.request('/health');
  }
}

export const api = new OlympusAPI();