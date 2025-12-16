# ROS 2 Educational Content API

This document describes the API contracts for the ROS 2 educational content system. Since this is primarily a documentation project, the API represents the interfaces for content access and validation.

## Content Access API

### Get Module Information
```
GET /api/v1/modules/ros2-nervous-system
```

**Description**: Retrieve information about the ROS 2 nervous system module.

**Response**:
```json
{
  "module_id": "ros2-nervous-system",
  "title": "ROS 2 Robotic Nervous System",
  "version": "1.0.0",
  "description": "Educational module covering ROS 2 fundamentals, Python integration, and humanoid modeling",
  "chapters": [
    {
      "id": "ros2-basics",
      "title": "ROS 2 Basics & Middleware",
      "sections": ["nodes", "topics", "services"]
    },
    {
      "id": "python-integration",
      "title": "Python Agents & ROS 2 Integration",
      "sections": ["rclpy", "agent-communication"]
    },
    {
      "id": "humanoid-modeling",
      "title": "Humanoid Robot Modeling with URDF",
      "sections": ["urdf-structure", "simulation-setup"]
    }
  ],
  "learning_objectives": [
    "Explain ROS 2 architecture",
    "Implement Python-ROS integration",
    "Create URDF robot models"
  ],
  "estimated_duration_hours": 8
}
```

### Get Chapter Content
```
GET /api/v1/modules/ros2-nervous-system/chapters/{chapter_id}
```

**Description**: Retrieve content for a specific chapter.

**Path Parameters**:
- `chapter_id`: The ID of the chapter (e.g., "ros2-basics", "python-integration", "humanoid-modeling")

**Response**:
```json
{
  "chapter_id": "ros2-basics",
  "title": "ROS 2 Basics & Middleware",
  "content": "Markdown content for the chapter...",
  "learning_objectives": ["Understand ROS 2 nodes", "Work with topics and services"],
  "prerequisites": ["basic-programming-knowledge"],
  "examples": [
    {
      "id": "publisher-subscriber",
      "title": "Basic Publisher-Subscriber",
      "type": "code_example",
      "language": "python",
      "content": "Python code for publisher subscriber example..."
    }
  ],
  "exercises": [
    {
      "id": "exercise-1",
      "title": "Create a simple publisher",
      "description": "Implement a ROS 2 publisher node",
      "difficulty": "beginner"
    }
  ]
}
```

### Validate Code Example
```
POST /api/v1/validation/code-examples
```

**Description**: Validate a code example against ROS 2 standards.

**Request Body**:
```json
{
  "language": "python",
  "content": "Python code content to validate",
  "context": "ROS 2 environment",
  "dependencies": ["rclpy", "std_msgs"]
}
```

**Response**:
```json
{
  "valid": true,
  "issues": [],
  "suggestions": [],
  "reproducible": true
}
```

### Get Citation Information
```
GET /api/v1/citations/{citation_id}
```

**Description**: Retrieve detailed information about a specific citation.

**Path Parameters**:
- `citation_id`: The unique identifier of the citation

**Response**:
```json
{
  "citation_id": "ros2-conceptual-paper-2023",
  "apa_citation": "Smith, J., & Doe, A. (2023). Conceptual foundations of ROS 2. Journal of Robotics, 15(3), 45-67.",
  "type": "peer_reviewed_article",
  "url": "https://doi.org/10.1234/example",
  "accessed_date": "2025-12-13",
  "verified": true
}
```

## Validation API

### Validate Content Quality
```
POST /api/v1/validation/content
```

**Description**: Validate content against quality standards (readability, accuracy, etc.).

**Request Body**:
```json
{
  "content": "Content to validate",
  "readability_target": "grade_10_to_12",
  "check_factual_accuracy": true,
  "verify_citations": true
}
```

**Response**:
```json
{
  "readability_score": 11.2,
  "factual_accuracy_verified": true,
  "citations_valid": true,
  "issues": [],
  "suggestions": ["Consider adding more examples for complex concepts"]
}
```

## Simulation API

### Get Simulation Example
```
GET /api/v1/simulations/{simulation_id}
```

**Description**: Retrieve a simulation example for educational use.

**Path Parameters**:
- `simulation_id`: The ID of the simulation example

**Response**:
```json
{
  "simulation_id": "basic-humanoid-walk",
  "title": "Basic Humanoid Walking Simulation",
  "description": "Simple simulation of a humanoid robot walking",
  "ros2_components": ["publisher", "subscriber", "service"],
  "urdf_files": ["humanoid_model.urdf"],
  "launch_file": "walk_simulation.launch.py",
  "instructions": "Detailed instructions for running the simulation",
  "expected_outcomes": ["Robot moves forward", "Joint positions updated correctly"]
}
```