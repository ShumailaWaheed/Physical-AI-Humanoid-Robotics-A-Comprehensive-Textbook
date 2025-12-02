from fastapi import APIRouter, HTTPException, Depends
from typing import List, Optional
import os
from openai import OpenAI

from app.qdrant_client import search_chunks # Import search_chunks from qdrant_client helper
from app.models import ChatRequest, ChatResponse # Import ChatRequest and ChatResponse from app.models

# Initialize OpenAI Client
try:
    openai_client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])
except KeyError:
    raise ValueError("OPENAI_API_KEY environment variable not set.")


router = APIRouter()

# Constants moved to qdrant_client.py where appropriate, keep LLM_MODEL here
EMBEDDING_MODEL = "text-embedding-3-small" # Kept here as it's for OpenAI embedding
LLM_MODEL = "gpt-3.5-turbo" # Or any other suitable OpenAI chat model

def generate_embedding_for_query(text: str) -> List[float]:
    """
    Generates an embedding for a single text query using OpenAI.
    """
    response = openai_client.embeddings.create(input=[text], model=EMBEDDING_MODEL)
    return response.data[0].embedding

def search_qdrant_for_chunks(query_embedding: List[float], limit: int = 5) -> List[dict]:
    """
    Searches Qdrant for similar chunks using the qdrant_client helper.
    Returns a list of payloads (dicts) from the retrieved points.
    """
    try:
        return search_chunks(query_embedding, limit)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to search Qdrant: {str(e)}")

def build_retrieval_prompt(question: str, chunks: List[dict], selected_text: Optional[str] = None) -> str:
    """
    Builds a prompt for the LLM using the retrieved chunks and optional selected text as context.
    """
    context_texts = [chunk.get("chunk_text", "") for chunk in chunks if chunk.get("chunk_text")]
    
    if selected_text:
        context_texts.insert(0, f"Selected Text: {selected_text}") # Prioritize selected text if provided

    if not context_texts:
        return f"Based on the available information, answer the following question: {question}"

    context_string = "\n\n".join(context_texts)
    
    prompt = f"""
You are a helpful assistant. Use the following pieces of context to answer the question at the end.
If you don't know the answer, just say that you don't know, don't try to make up an answer.

---
Context:
{context_string}
---

Question: {question}
"""
    return prompt

def get_answer_from_llm(prompt: str) -> str:
    """
    Gets an answer from the OpenAI LLM based on the generated prompt.
    """
    try:
        response = openai_client.chat.completions.create(
            model=LLM_MODEL,
            messages=[
                {"role": "system", "content": "You are a helpful assistant."}, 
                {"role": "user", "content": prompt}
            ],
            max_tokens=500, # Adjust as needed
        )
        return response.choices[0].message.content.strip()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get answer from LLM: {str(e)}")


@router.post("/chat", response_model=ChatResponse, summary="Chat with the RAG service")
async def chat_with_rag(request: ChatRequest):
    """
    Takes a user question, embeds it, fetches similar chunks from Qdrant,
    builds a retrieval prompt, and returns an answer from the LLM along with source paths.
    """
    # 1. Embed question
    query_embedding = generate_embedding_for_query(request.question)

    # 2. Fetch top similar chunks from Qdrant
    retrieved_chunks_payloads = search_qdrant_for_chunks(query_embedding, limit=6) # Top 6 chunks

    # 3. Build retrieval prompt, including selected text if present
    retrieval_prompt = build_retrieval_prompt(request.question, retrieved_chunks_payloads, request.selected_text)

    # 4. Get answer from LLM
    answer = get_answer_from_llm(retrieval_prompt)

    # 5. Extract unique source book_ids
    sources = list(set([
        payload["book_id"] 
        for payload in retrieved_chunks_payloads 
        if "book_id" in payload
    ]))

    return ChatResponse(answer=answer, sources=sources)

