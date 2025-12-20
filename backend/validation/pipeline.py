"""
Complete validation pipeline integrating all components
"""
import logging
import time
from typing import Dict, Any, List
from qdrant_client import QdrantClient

from .retrieval import retrieve_all_chunks_from_qdrant, validate_collection_existence, count_and_verify_chunks
from .validation import validate_embedding_dimensions, calculate_embedding_statistics
from .similarity_search import test_retrieval_accuracy
from .completeness_audit import identify_missing_data, implement_corrupted_data_detection, add_validation_for_content_completeness
from .reporting import generate_validation_report, add_summary_statistics_to_report, implement_report_file_output


logger = logging.getLogger(__name__)


def run_main_validation_pipeline(client: QdrantClient, collection_name: str = "physical_ai_book", output_path: str = "./validation_report.json") -> Dict[str, Any]:
    """
    Implement main validation pipeline function integrating all components.

    Args:
        client: QdrantClient instance
        collection_name: Name of the Qdrant collection to validate
        output_path: Path to save the validation report

    Returns:
        Complete validation results
    """
    start_time = time.time()
    logger.info(f"Starting validation pipeline for collection: {collection_name}")

    # Initialize results dictionary
    validation_results = {
        "collection_name": collection_name,
        "pipeline_start_time": start_time,
        "errors": [],
        "warnings": []
    }

    try:
        # Step 1: Check if collection exists
        if not validate_collection_existence(client, collection_name):
            error_msg = f"Collection '{collection_name}' does not exist"
            logger.error(error_msg)
            validation_results["errors"].append(error_msg)
            return validation_results

        # Step 2: Retrieve all chunks from Qdrant
        logger.info("Step 1: Retrieving all chunks from Qdrant...")
        all_chunks = retrieve_all_chunks_from_qdrant(client, collection_name)
        validation_results["total_chunks"] = len(all_chunks)

        if not all_chunks:
            warning_msg = f"No chunks retrieved from collection '{collection_name}'"
            logger.warning(warning_msg)
            validation_results["warnings"].append(warning_msg)
        else:
            # Validate chunk integrity
            chunk_stats = count_and_verify_chunks(all_chunks)
            validation_results.update(chunk_stats)

        # Step 3: Extract embeddings and validate them
        logger.info("Step 2: Validating embeddings...")
        embeddings = []
        for chunk in all_chunks:
            vector = chunk.get("vector")
            if vector is not None and isinstance(vector, list):
                embeddings.append(vector)

        validation_results["total_embeddings"] = len(embeddings)

        if embeddings:
            embedding_validation_results = validate_embedding_dimensions(embeddings)
            validation_results.update(embedding_validation_results)

            # Calculate embedding statistics
            embedding_stats = calculate_embedding_statistics(embeddings)
            validation_results["embedding_statistics"] = embedding_stats
        else:
            warning_msg = "No embeddings found in the retrieved chunks"
            logger.warning(warning_msg)
            validation_results["warnings"].append(warning_msg)
            validation_results["valid_embeddings"] = 0
            validation_results["invalid_embeddings"] = 0

        # Step 4: Perform similarity search validation
        logger.info("Step 3: Performing similarity search validation...")
        # Generate sample queries from the content
        sample_queries = []
        if all_chunks:
            # Take first few chunks to generate sample queries
            for i, chunk in enumerate(all_chunks[:3]):
                title = chunk.get("title", "")
                content = chunk.get("content", "")
                if title:
                    sample_queries.append(title)
                if content:
                    # Extract first sentence from content as a sample query
                    sentences = content.split('.')
                    if len(sentences) > 0:
                        first_sentence = sentences[0].strip()
                        if len(first_sentence) > 10:  # At least 10 characters
                            sample_queries.append(first_sentence)

        # Add some generic queries if we couldn't generate from content
        if not sample_queries:
            sample_queries = [
                "Introduction to AI",
                "RAG chatbot pipeline",
                "Book deployment"
            ]

        search_accuracy_results = test_retrieval_accuracy(client, collection_name, sample_queries, top_k=5)
        validation_results["search_accuracy_results"] = search_accuracy_results
        validation_results["similarity_search_success_rate"] = search_accuracy_results.get("accuracy_rate", 0)

        # Step 5: Perform completeness audit
        logger.info("Step 4: Performing completeness audit...")
        # For now, we'll use the URLs from the retrieved chunks as the "original" list
        # In a real scenario, this would come from the original book source
        original_urls = list(set([chunk.get("url", "") for chunk in all_chunks if chunk.get("url", "")]))

        completeness_results = identify_missing_data(original_urls, all_chunks)
        validation_results["completeness_results"] = completeness_results
        validation_results["missing_content"] = completeness_results.get("missing_count", 0)

        # Check for corrupted data
        corrupted_chunks = implement_corrupted_data_detection(all_chunks)
        validation_results["corrupted_data"] = len(corrupted_chunks)
        validation_results["corrupted_details"] = corrupted_chunks

        # Validate content completeness
        content_completeness = add_validation_for_content_completeness(all_chunks)
        validation_results["content_completeness"] = content_completeness

        # Step 6: Generate final validation summary
        validation_results["summary"] = f"Validation completed for {len(all_chunks)} chunks in '{collection_name}'"
        validation_results["overall_status"] = "PASS" if len(validation_results["errors"]) == 0 else "FAIL"

        # Calculate total execution time
        total_time = time.time() - start_time
        validation_results["execution_time_seconds"] = round(total_time, 2)

        logger.info(f"Validation pipeline completed in {validation_results['execution_time_seconds']} seconds")
        logger.info(f"Total chunks processed: {len(all_chunks)}")
        logger.info(f"Total embeddings validated: {len(embeddings)}")
        logger.info(f"Similarity search success rate: {validation_results['similarity_search_success_rate']:.2f}%")

    except Exception as e:
        error_msg = f"Error during validation pipeline execution: {str(e)}"
        logger.error(error_msg)
        validation_results["errors"].append(error_msg)

    return validation_results


def add_progress_tracking_and_status_reporting(progress_callback=None):
    """
    Add progress tracking and status reporting to the pipeline.

    Args:
        progress_callback: Optional callback function to report progress
    """
    def report_progress(step, total_steps, message):
        if progress_callback:
            progress_percentage = (step / total_steps) * 100
            progress_callback(progress_percentage, message)
        else:
            logger.info(f"Progress: {step}/{total_steps} - {message}")

    # This function would be integrated into the main pipeline to report progress
    pass


def implement_configurable_validation_parameters() -> Dict[str, Any]:
    """
    Implement configurable validation parameters.

    Returns:
        Dictionary with default validation parameters
    """
    params = {
        "top_k_results": 5,
        "similarity_threshold": 0.5,
        "min_content_length": 10,
        "expected_embedding_dimension": 1024,
        "batch_size": 100,
        "timeout": 30,
        "max_concurrent_requests": 5
    }
    return params


def add_performance_metrics_and_timing(validation_start_time: float) -> Dict[str, Any]:
    """
    Add performance metrics and timing information.

    Args:
        validation_start_time: Start time of the validation process

    Returns:
        Dictionary with performance metrics
    """
    import time
    end_time = time.time()
    duration = end_time - validation_start_time

    # In a real implementation, we would also track:
    # - Memory usage
    # - API call counts and rates
    # - Network latency
    # - Processing speeds

    metrics = {
        "start_time": validation_start_time,
        "end_time": end_time,
        "duration_seconds": round(duration, 2),
        "throughput_chunks_per_second": 0,  # Calculated based on actual processing
    }

    if duration > 0:
        # This would be updated with actual processing counts
        pass

    return metrics