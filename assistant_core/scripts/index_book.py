import os
import argparse
from typing import List, Dict, Any
# from openai import OpenAI # Removed OpenAI
import google.generativeai as genai # Added Gemini import
from qdrant_client import models
from langchain_text_splitters import RecursiveCharacterTextSplitter

# Import Qdrant helper and constants
from app.qdrant_client import upsert_chunks, ensure_collection_exists, COLLECTION_NAME, EMBEDDING_DIMENSION

# --- Gemini API Configuration ---
# Where Gemini API is called: Configuration for the Gemini API.
try:
    genai.configure(api_key=os.environ["GEMINI_API_KEY"])
except KeyError:
    raise ValueError("GEMINI_API_KEY environment variable not set. Please set it before running the script.")

# EMBEDDING_MODEL for Gemini
EMBEDDING_MODEL = "models/embedding-001" 
# EMBEDDING_DIMENSION is 768 for models/embedding-001, updated in qdrant_client.py

def get_markdown_files(directory: str) -> List[str]:
    """
    Recursively finds all markdown (.md) files in a given directory.
    """
    markdown_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".md"):
                markdown_files.append(os.path.join(root, file))
    return markdown_files

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

def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """
    Generates embeddings for a list of texts in a batch using Gemini.
    Where Gemini API is called: Using genai.embed_content for embedding generation.
    """
    embeddings = []
    for text in texts:
        response = genai.embed_content(model=EMBEDDING_MODEL, content=text)
        embeddings.append(response['embedding'])
    return embeddings

def index_book_content(source_dir: str):
    """
    Reads markdown files from source_dir, chunks them, generates embeddings in batches,
    and upserts them into the Qdrant Cloud collection.
    """
    print(f"Starting indexing process for Markdown files in: {source_dir}")

    # Ensure Qdrant collection exists (Qdrant keys used internally by this function)
    ensure_collection_exists()

    markdown_files = get_markdown_files(source_dir)
    if not markdown_files:
        print(f"No Markdown files found in {source_dir}. Exiting.")
        return

    for file_path in markdown_files:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        book_id = os.path.basename(file_path) # Use filename as book_id
        chunks = get_text_chunks(content)

        print(f"Processing '{book_id}': {len(chunks)} chunks generated.")

        # Generate embeddings in batches and upsert to Qdrant
        # Batch size for Gemini embedding might need tuning
        batch_size = 100 
        for i in range(0, len(chunks), batch_size):
            chunk_batch = chunks[i : i + batch_size]
            embeddings_batch = generate_embeddings_batch(chunk_batch)

            points = []
            for j, chunk in enumerate(chunk_batch):
                global_chunk_index = i + j
                points.append(
                    models.PointStruct(
                        id=f"{book_id}-{global_chunk_index}", # Unique ID for each chunk
                        vector=embeddings_batch[j],
                        payload={"book_id": book_id, "chunk_text": chunk, "chunk_index": global_chunk_index},
                    )
                )
            
            if points:
                print(f"Upserting batch of {len(points)} points for '{book_id}'...")
                # Where Qdrant keys are used: upsert_chunks uses QDRANT_HOST and QDRANT_API_KEY.
                upsert_chunks(points) 
                print(f"Batch upserted for '{book_id}'.")

    print(f"Indexing complete for {len(markdown_files)} files.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Index Markdown textbook files into Qdrant Cloud.")
    parser.add_argument(
        "--source_dir",
        type=str,
        required=True,
        help="Path to the directory containing textbook Markdown files.",
    )
    args = parser.parse_args()

    index_book_content(args.source_dir)
