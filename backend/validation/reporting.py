"""
Functions for generating validation reports
"""
import json
import logging
from datetime import datetime
from typing import Dict, Any, List
from pathlib import Path


logger = logging.getLogger(__name__)


def generate_validation_report(validation_results: Dict[str, Any], output_path: str = None) -> str:
    """
    Generate a validation report with counts of stored items and any issues found.

    Args:
        validation_results: Results from various validation steps
        output_path: Optional path to save the report file

    Returns:
        JSON string of the validation report
    """
    report = {
        "id": f"validation_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        "timestamp": datetime.now().isoformat(),
        "collection_name": validation_results.get("collection_name", "unknown"),
        "total_chunks": validation_results.get("total_chunks", 0),
        "total_embeddings": validation_results.get("total_embeddings", 0),
        "valid_embeddings": validation_results.get("valid_embeddings", 0),
        "invalid_embeddings": validation_results.get("invalid_embeddings", 0),
        "missing_content": validation_results.get("missing_content", 0),
        "corrupted_data": validation_results.get("corrupted_data", 0),
        "similarity_search_success_rate": validation_results.get("similarity_search_success_rate", 0.0),
        "issues": validation_results.get("issues", []),
        "summary": validation_results.get("summary", "Validation completed"),
        "details": validation_results.get("details", {})
    }

    # Convert to JSON string
    report_json = json.dumps(report, indent=2)

    # Save to file if output path provided
    if output_path:
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(report_json)
        logger.info(f"Validation report saved to {output_path}")

    logger.info(f"Validation report generated: {report['summary']}")
    return report_json


def design_json_report_structure() -> Dict[str, Any]:
    """
    Design JSON report structure based on data model.

    Returns:
        Dictionary representing the report structure
    """
    # This function defines the structure of the validation report
    # based on the data model specified in the requirements
    structure = {
        "id": "string (unique identifier for the validation run)",
        "timestamp": "datetime (when validation was performed)",
        "collection_name": "string (name of the Qdrant collection validated)",
        "total_chunks": "integer (total number of chunks found in collection)",
        "total_embeddings": "integer (total number of embeddings validated)",
        "valid_embeddings": "integer (number of embeddings with correct dimensions)",
        "invalid_embeddings": "integer (number of embeddings with incorrect dimensions)",
        "missing_content": "integer (number of entries with missing content)",
        "corrupted_data": "integer (number of entries with corrupted data)",
        "similarity_search_success_rate": "float (percentage of successful similarity searches)",
        "issues": "array<object> (detailed list of identified issues)",
        "summary": "string (overall pass/fail status)"
    }

    return structure


def add_summary_statistics_to_report(validation_results: Dict[str, Any]) -> Dict[str, Any]:
    """
    Add summary statistics to report.

    Args:
        validation_results: Results from various validation steps

    Returns:
        Updated validation results with summary statistics
    """
    # Calculate additional summary statistics
    total_embeddings = validation_results.get("total_embeddings", 0)
    valid_embeddings = validation_results.get("valid_embeddings", 0)
    invalid_embeddings = validation_results.get("invalid_embeddings", 0)

    if total_embeddings > 0:
        embedding_validation_rate = (valid_embeddings / total_embeddings) * 100
    else:
        embedding_validation_rate = 0

    validation_results["embedding_validation_rate"] = embedding_validation_rate

    # Add more summary statistics
    total_chunks = validation_results.get("total_chunks", 0)
    missing_content = validation_results.get("missing_content", 0)
    corrupted_data = validation_results.get("corrupted_data", 0)

    if total_chunks > 0:
        data_integrity_rate = ((total_chunks - missing_content - corrupted_data) / total_chunks) * 100
    else:
        data_integrity_rate = 0

    validation_results["data_integrity_rate"] = data_integrity_rate

    return validation_results


def include_detailed_issue_listings_in_report(issues: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Include detailed issue listings in report.

    Args:
        issues: List of issues to include in the report

    Returns:
        List of issues with detailed information
    """
    # The issues are already detailed from other validation functions
    # This function can be used to format or enhance the issue listings
    formatted_issues = []
    for issue in issues:
        formatted_issue = {
            "id": issue.get("id", "unknown"),
            "type": issue.get("type", "general"),
            "description": issue.get("description", "No description provided"),
            "severity": issue.get("severity", "medium"),
            "timestamp": issue.get("timestamp", datetime.now().isoformat()),
            "details": issue.get("details", {})
        }
        formatted_issues.append(formatted_issue)

    return formatted_issues


def implement_report_file_output(report_data: Dict[str, Any], output_path: str = "./validation_report.json") -> bool:
    """
    Implement report file output.

    Args:
        report_data: The report data to save
        output_path: Path to save the report file

    Returns:
        True if successful, False otherwise
    """
    try:
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)

        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)

        logger.info(f"Validation report saved to {output_path}")
        return True
    except Exception as e:
        logger.error(f"Failed to save validation report to {output_path}: {e}")
        return False


def create_validation_summary(retrieval_results: Dict[str, Any],
                             embedding_results: Dict[str, Any],
                             search_results: Dict[str, Any],
                             completeness_results: Dict[str, Any]) -> Dict[str, Any]:
    """
    Create a comprehensive validation summary from all validation results.

    Args:
        retrieval_results: Results from data retrieval validation
        embedding_results: Results from embedding validation
        search_results: Results from similarity search validation
        completeness_results: Results from completeness audit

    Returns:
        Comprehensive validation summary
    """
    summary = {
        "collection_name": retrieval_results.get("collection_name", "unknown"),
        "total_chunks": retrieval_results.get("total_count", 0),
        "retrieval_success": retrieval_results.get("success", True),
        "valid_embeddings": embedding_results.get("valid_embeddings", 0),
        "invalid_embeddings": embedding_results.get("invalid_embeddings", 0),
        "embedding_validation_success_rate": embedding_results.get("valid_percentage", 0.0),
        "search_accuracy_rate": search_results.get("accuracy_rate", 0.0),
        "missing_entries": completeness_results.get("missing_count", 0),
        "corrupted_entries": completeness_results.get("corrupted_count", 0),
        "overall_status": "PASS" if (
            retrieval_results.get("success", False) and
            embedding_results.get("valid_percentage", 0) >= 95 and
            search_results.get("accuracy_rate", 0) >= 90
        ) else "FAIL",
        "recommendations": []
    }

    # Add recommendations based on results
    if embedding_results.get("valid_percentage", 0) < 95:
        summary["recommendations"].append("Embedding validation failed threshold, check embedding generation process")

    if search_results.get("accuracy_rate", 0) < 90:
        summary["recommendations"].append("Similarity search accuracy below threshold, review embedding quality")

    if completeness_results.get("missing_count", 0) > 0:
        summary["recommendations"].append(f"Found {completeness_results['missing_count']} missing entries, check ingestion pipeline")

    return summary


def log_validation_results(validation_results: Dict[str, Any]):
    """
    Log validation results in a structured way.

    Args:
        validation_results: Results to log
    """
    logger.info("=== VALIDATION RESULTS ===")
    logger.info(f"Collection: {validation_results.get('collection_name', 'unknown')}")
    logger.info(f"Total chunks retrieved: {validation_results.get('total_chunks', 0)}")
    logger.info(f"Valid embeddings: {validation_results.get('valid_embeddings', 0)}")
    logger.info(f"Invalid embeddings: {validation_results.get('invalid_embeddings', 0)}")
    logger.info(f"Embedding validation success rate: {validation_results.get('embedding_validation_success_rate', 0.0):.2f}%")
    logger.info(f"Search accuracy rate: {validation_results.get('search_accuracy_rate', 0.0):.2f}%")
    logger.info(f"Missing entries: {validation_results.get('missing_entries', 0)}")
    logger.info(f"Corrupted entries: {validation_results.get('corrupted_entries', 0)}")
    logger.info(f"Overall status: {validation_results.get('overall_status', 'UNKNOWN')}")

    recommendations = validation_results.get("recommendations", [])
    if recommendations:
        logger.info("Recommendations:")
        for rec in recommendations:
            logger.info(f"  - {rec}")

    logger.info("=========================")