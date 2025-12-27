from typing import List, Dict, Optional
import logging
import os
from groq import Groq
from dotenv import load_dotenv
import json

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class GroqRAGAgent:
    def __init__(self):
        """
        Initialize the Groq RAG Agent with Groq API and retrieval capabilities
        """
        self.groq_api_key = os.getenv('GROQ_API_KEY')
        self.groq_client = Groq(api_key=self.groq_api_key) if self.groq_api_key else None

        # Initialize retrieval components
        self.retriever = self._initialize_retriever()

        # Import search service
        try:
            from ..search_service import search_book_content
            self.search_book_content = search_book_content
        except ImportError:
            logger.warning("Search service not available, using mock implementation")
            self.search_book_content = self._mock_search

    def _mock_search(self, query: str, search_type: str = "vector", top_k: int = 5) -> List[Dict]:
        """
        Mock search implementation when the search service is not available
        """
        # Return mock content based on query
        if "ai" in query.lower():
            return [
                {
                    'content': 'Artificial Intelligence (AI) refers to computer systems designed to perform tasks that typically require human intelligence, such as learning, reasoning, problem-solving, and understanding natural language. In Physical AI, these systems interact with the physical world.',
                    'url': '/module-1/ai-concepts',
                    'title': 'AI Concepts in Physical AI'
                }
            ]
        elif "robot" in query.lower() or "humanoid" in query.lower():
            return [
                {
                    'content': 'Humanoid robots are designed with a human-like body structure, including limbs and often a head, to interact effectively with human environments and tools.',
                    'url': '/module-2/humanoid-design',
                    'title': 'Humanoid Robot Design Principles'
                }
            ]
        else:
            return [
                {
                    'content': 'Physical AI and Humanoid Robotics is an interdisciplinary field combining artificial intelligence with robotics to create systems that can interact with the physical world in human-like ways.',
                    'url': '/module-1/introduction',
                    'title': 'Introduction to Physical AI and Humanoid Robotics'
                }
            ]

    def _initialize_retriever(self):
        """
        Initialize the retrieval component to connect with Qdrant
        This connects to the existing Qdrant database with Cohere embeddings
        """
        try:
            from qdrant_client import QdrantClient

            # Connect to existing Qdrant instance
            qdrant_client = QdrantClient(
                url=os.getenv('QDRANT_URL'),
                api_key=os.getenv('QDRANT_API_KEY')
            )

            collection_name = os.getenv('QDRANT_COLLECTION_NAME', 'book_content')

            # Verify connection
            collections = qdrant_client.get_collections()
            if collection_name not in [c.name for c in collections.collections]:
                logger.warning(f"Collection {collection_name} not found in Qdrant")
                return None

            return qdrant_client, collection_name
        except Exception as e:
            logger.error(f"Error initializing retriever: {str(e)}")
            return None

    def get_response(self, query: str, context: Optional[str] = None, target_language: str = "english") -> Dict:
        """
        Get response from Groq using RAG approach
        """
        try:
            # Retrieve relevant content based on the query
            retrieved_content = self.search_book_content(query=query, search_type="hybrid", top_k=5)

            # Prepare the prompt for Groq
            prompt = self._build_prompt(query, retrieved_content, context)

            # Generate response using Groq API
            response = self._call_groq_api(prompt)

            # Translate response if needed
            if target_language.lower() in ["roman_urdu", "romanurdu", "urdu"]:
                try:
                    from ..translation_service import translate_text
                    translation_result = translate_text(response, target_language="roman_urdu")
                    if translation_result.success:
                        response = translation_result.translated_text
                except ImportError:
                    logger.warning("Translation service not available, returning original response")

            # Prepare sources information
            sources = self._extract_sources(retrieved_content)

            return {
                'response': response,
                'sources': sources,
                'target_language': target_language
            }
        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return {
                'response': "I'm having trouble processing your query right now. Please try again later.",
                'sources': [],
                'target_language': target_language
            }

    def search_content(self, query: str, search_type: str = "hybrid", top_k: int = 5) -> List[Dict]:
        """
        Search book content using the search service
        """
        try:
            results = self.search_book_content(query=query, search_type=search_type, top_k=top_k)
            return results
        except Exception as e:
            logger.error(f"Error searching content: {str(e)}")
            return []

    def translate_text(self, text: str, target_language: str = "roman_urdu") -> Dict:
        """
        Translate text using the translation service
        """
        try:
            from ..translation_service import translate_text as translate_func
            result = translate_func(text, target_language)
            return {
                'original_text': result.original_text,
                'translated_text': result.translated_text,
                'success': result.success,
                'error': result.error
            }
        except ImportError:
            logger.error("Translation service not available")
            return {
                'original_text': text,
                'translated_text': "",
                'success': False,
                'error': "Translation service not available"
            }

    def _call_groq_api(self, prompt: str) -> str:
        """
        Call the Groq API to generate a response
        """
        try:
            # Check if Groq API key is available
            if not self.groq_client:
                logger.warning("Groq API key not configured, using fallback response")
                if "hi" in prompt.lower() or "hello" in prompt.lower():
                    return "Hello! I'm your Groq RAG Assistant. I can help you with questions about the book content. What would you like to know?"
                elif "ai" in prompt.lower():
                    return "AI (Artificial Intelligence) refers to computer systems designed to perform tasks that typically require human intelligence, such as learning, reasoning, problem-solving, and understanding natural language. In the context of Physical AI, it involves systems that interact with the physical world."
                else:
                    return f"I received your query: '{prompt[:50]}...'. Please configure your GROQ_API_KEY in the .env file to get full AI responses. Currently using fallback responses."

            # Call Groq API with the prompt
            chat_completion = self.groq_client.chat.completions.create(
                messages=[
                    {
                        "role": "user",
                        "content": prompt,
                    }
                ],
                model="llama3-70b-8192",  # Using a powerful model; can be changed as needed
                temperature=0.7,
                max_tokens=500
            )

            # Extract the generated text from the response
            return chat_completion.choices[0].message.content
        except Exception as e:
            logger.error(f"Error calling Groq API: {str(e)}")
            # Fallback response when API is not available
            if "hi" in prompt.lower() or "hello" in prompt.lower():
                return "Hello! I'm your Groq RAG Assistant. I can help you with questions about the book content. What would you like to know?"
            elif "ai" in prompt.lower():
                return "AI (Artificial Intelligence) refers to computer systems designed to perform tasks that typically require human intelligence, such as learning, reasoning, problem-solving, and understanding natural language. In the context of Physical AI, it involves systems that interact with the physical world."
            else:
                return f"I received your query: '{prompt[:50]}...'. Currently, I'm using a fallback response system. In a full implementation, I would retrieve relevant information from the book content and generate a response using Groq."

    def _build_prompt(self, query: str, retrieved_content: List, context: Optional[str] = None) -> str:
        """
        Build the prompt for Qwen with retrieved content
        """
        prompt_parts = []

        # Add system context
        prompt_parts.append("You are an AI assistant for a Physical AI and Humanoid Robotics book. Answer questions based on the provided content.")

        # Add retrieved content if available
        if retrieved_content:
            prompt_parts.append("\nRelevant content from the book:")
            for i, chunk in enumerate(retrieved_content[:3]):  # Use top 3 chunks
                content_text = chunk.get('content', '') if isinstance(chunk, dict) else str(chunk)
                prompt_parts.append(f"\nContent {i+1}: {content_text}")

        # Add user query
        prompt_parts.append(f"\nUser query: {query}")

        # Add context if provided
        if context:
            prompt_parts.append(f"\nPage context: {context}")

        # Add instruction for response
        prompt_parts.append("\nPlease provide a helpful and accurate response based on the above information.")

        return "\n".join(prompt_parts)

    def _extract_sources(self, retrieved_content: List) -> List[Dict]:
        """
        Extract source information from retrieved content
        """
        sources = []
        for chunk in retrieved_content:
            if isinstance(chunk, dict):
                source_info = {
                    'url': chunk.get('url', ''),
                    'title': chunk.get('title', ''),
                    'content_snippet': chunk.get('content', '')[:200] + '...' if len(chunk.get('content', '')) > 200 else chunk.get('content', '')
                }
                sources.append(source_info)

        return sources

# Initialize the agent
groq_rag_agent = GroqRAGAgent()