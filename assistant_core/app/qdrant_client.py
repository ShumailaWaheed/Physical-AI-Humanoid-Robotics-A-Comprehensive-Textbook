import os
from typing import List, Dict, Any
from qdrant_client import QdrantClient, models

# Initialize Qdrant Client
try:
    QDRANT_HOST = os.environ["QDRANT_HOST"]
    QDRANT_API_KEY = os.environ["QDRANT_API_KEY"]
except KeyError:
    raise ValueError("QDRANT_HOST and QDRANT_API_KEY environment variables not set.")

qdrant_client = QdrantClient(
    host=QDRANT_HOST, 
    api_key=QDRANT_API_KEY
)

COLLECTION_NAME = "book_chunks"
EMBEDDING_DIMENSION = 768 

def ensure_collection_exists():
    """
    Ensures that the Qdrant collection exists with the correct vector parameters.
    Where Qdrant keys are used: QDRANT_HOST and QDRANT_API_KEY from environment variables are used by qdrant_client.
    """
    current_collections = qdrant_client.get_collections().collections
    if not any(c.name == COLLECTION_NAME for c in current_collections):
        print(f"Collection '{COLLECTION_NAME}' not found, creating it...")
        qdrant_client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=EMBEDDING_DIMENSION, distance=models.Distance.COSINE),
        )
        print(f"Collection '{COLLECTION_NAME}' created.")
    else:
        print(f"Collection '{COLLECTION_NAME}' already exists.")


def search_chunks(query_vector: List[float], limit: int = 6) -> List[Dict[str, Any]]:
    """
    Searches the 'book_chunks' collection in Qdrant for similar vectors.

    Args:
        query_vector: The embedding vector of the query.
        limit: The maximum number of similar chunks to retrieve.

    Returns:
        A list of dictionaries, where each dictionary contains the chunk's text,
        score, and book_id.
    """
    search_result = qdrant_client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_vector,
        limit=limit,
        append_payload=True,
    )

    results = []
    for hit in search_result:
        if hit.payload:
            results.append(
                {
                    "chunk_text": hit.payload.get("chunk_text"),
                    "score": hit.score,
                    "book_id": hit.payload.get("book_id"),
                }
            )
    return results

def upsert_chunks(points: List[models.PointStruct]):
    """
    Upserts a list of PointStruct objects into the 'book_chunks' collection.
    """
    ensure_collection_exists() # Ensure collection exists before upserting
    operation_info = qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        wait=True,
        points=points,
    )
    return operation_info
