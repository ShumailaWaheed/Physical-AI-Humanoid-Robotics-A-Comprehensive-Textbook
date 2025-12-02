from fastapi import APIRouter, HTTPException, Depends
from typing import List, Optional
import os
import tiktoken
from openai import OpenAI
from bs4 import BeautifulSoup
from langchain.text_splitter import RecursiveCharacterTextSplitter

from app.qdrant_client import upsert_chunks, COLLECTION_NAME, EMBEDDING_DIMENSION, models # Import qdrant helper functions and constants
from app.models import IngestRequest # Import IngestRequest from app.models

# Initialize OpenAI Client
# Ensure OPENAI_API_KEY is set in your environment variables
try:
    openai_client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])
except KeyError:
    raise ValueError("OPENAI_API_KEY environment variable not set.")


router = APIRouter()

# EMBEDDING_MODEL is kept here as it's specific to OpenAI interaction
EMBEDDING_MODEL = "text-embedding-3-small" 

def get_text_chunks(text: str, chunk_size: int = 800, chunk_overlap: int = 100) -> List[str]:
    """
    Splits text into chunks using RecursiveCharacterTextSplitter.
    """
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=chunk_size,
        chunk_overlap=chunk_overlap,
        length_function=len,
        is_separator_regex=False,
    )
    return text_splitter.split_text(text)

def clean_html(html_content: str) -> str:
    """Removes HTML tags and returns clean text."""
    soup = BeautifulSoup(html_content, 'html.parser')
    return soup.get_text(separator=' ', strip=True)

def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generates embeddings for a list of texts using OpenAI.
    """
    response = openai_client.embeddings.create(input=texts, model=EMBEDDING_MODEL)
    return [embedding.embedding for embedding in response.data]

@router.post("/ingest/book", summary="Ingest book content and create embeddings")
async def ingest_book_content(request: IngestRequest):
    """
    Reads textbook content (Markdown or HTML), chunks it, generates embeddings,
    and upserts them into the Qdrant Cloud collection.
    """
    
    # 1. Prepare content for chunking
    if request.content_type == "html":
        processed_content = clean_html(request.content)
    elif request.content_type == "markdown":
        # For markdown, we can directly chunk, but might need more sophisticated cleaning
        # if there are specific markdown elements to ignore. For now, treat as plain text.
        processed_content = request.content
    else:
        raise HTTPException(status_code=400, detail="Invalid content_type. Must be 'markdown' or 'html'.")

    # 2. Chunk text
    chunks = get_text_chunks(processed_content)
    if not chunks:
        raise HTTPException(status_code=400, detail="No text chunks generated from the content.")

    # 3. Generate embeddings for chunks
    embeddings = generate_embeddings(chunks)
    if len(embeddings) != len(chunks):
        raise HTTPException(status_code=500, detail="Mismatch between number of chunks and embeddings generated.")

    # 4. Upsert into Qdrant using the helper
    points = []
    for i, chunk in enumerate(chunks):
        points.append(
            models.PointStruct(
                id=f"{request.book_id}-{i}", # Unique ID for each chunk
                vector=embeddings[i],
                payload={"book_id": request.book_id, "chunk_text": chunk, "chunk_index": i},
            )
        )
    
    try:
        operation_info = upsert_chunks(points)
        
        return {
            "message": f"Successfully ingested {len(chunks)} chunks for book '{request.book_id}'",
            "qdrant_operation_info": operation_info.dict(),
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to upsert into Qdrant: {str(e)}")

