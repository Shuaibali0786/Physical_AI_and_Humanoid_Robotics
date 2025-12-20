// Type definitions for RAG Chat component

export interface UserQuery {
  text: string;
  timestamp: number;
  pageContext?: PageContext;
  metadata?: Record<string, any>;
}

export interface AgentResponse {
  answer: string;
  query: string;
  retrievedChunks: RetrievedChunk[];
  sources: string[];
  confidence: number;
  timestamp: number;
  error?: ErrorInfo;
}

export interface PageContext {
  url: string;
  title: string;
  contentSnippet?: string;
  section?: string;
}

export interface ChatSession {
  id: string;
  messages: ChatMessage[];
  isLoading: boolean;
  error?: ErrorInfo;
  pageContext?: PageContext;
}

export interface ChatMessage {
  id: string;
  text: string;
  sender: 'user' | 'agent';
  timestamp: number;
  sources?: string[];
}

export interface RetrievedChunk {
  id: string;
  content: string;
  score: number;
  url: string;
  title: string;
  metadata: Record<string, any>;
}

export interface ErrorInfo {
  type: string; // 'network', 'api', 'validation', etc.
  message: string;
  timestamp: number;
  details?: Record<string, any>;
}

export interface APICommunication {
  endpoint: string;
  method: string;
  headers: Record<string, string>;
  requestBody: Record<string, any>;
  response: Record<string, any>;
  status: number;
  timestamp: number;
}

export interface QueryRequest {
  query: string;
  max_results?: number;
  temperature?: number;
  metadata_filters?: Record<string, any>;
  pageContext?: PageContext;
}

export interface QueryResponse {
  answer: string;
  query: string;
  retrieved_chunks: RetrievedChunk[];
  sources: string[];
  confidence: number;
  timestamp: number;
}