import { QueryRequest, QueryResponse, PageContext } from '../types/rag-chat.types';

const DEFAULT_API_BASE_URL = process.env.REACT_APP_RAG_API_URL || 'http://localhost:8000';

class ApiService {
  private baseUrl: string;

  constructor(baseUrl?: string) {
    this.baseUrl = baseUrl || DEFAULT_API_BASE_URL;
  }

  async queryAgent(
    query: string,
    pageContext?: PageContext,
    maxResults: number = 5,
    temperature: number = 0.7
  ): Promise<QueryResponse> {
    const request: QueryRequest = {
      query,
      max_results: maxResults,
      temperature,
      metadata_filters: {},
      ...(pageContext && { pageContext })
    };

    try {
      const response = await fetch(`${this.baseUrl}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} - ${response.statusText}`);
      }

      const data: QueryResponse = await response.json();
      return data;
    } catch (error) {
      console.error('Error querying RAG agent:', error);
      throw error;
    }
  }

  async checkHealth(): Promise<boolean> {
    try {
      const response = await fetch(`${this.baseUrl}/health`);
      return response.ok;
    } catch (error) {
      console.error('Error checking API health:', error);
      return false;
    }
  }

  async getConfig(): Promise<any> {
    try {
      const response = await fetch(`${this.baseUrl}/config`);
      if (!response.ok) {
        throw new Error(`Config API error: ${response.status}`);
      }
      return await response.json();
    } catch (error) {
      console.error('Error fetching config:', error);
      throw error;
    }
  }
}

export default new ApiService();