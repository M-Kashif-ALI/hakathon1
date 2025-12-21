# Data Model: Vision-Language-Action (VLA) Integration

## Overview
This document defines the conceptual data models relevant to the Vision-Language-Action (VLA) integration module, focusing on the educational content structure rather than runtime data models since this is primarily a documentation module.

## Educational Content Entities

### 1. VLA Module
- **Description**: Top-level module containing all VLA-related educational content
- **Fields**:
  - id: string (unique identifier for the module)
  - title: string (e.g., "Vision-Language-Action Integration")
  - description: string (overview of VLA concepts)
  - targetAudience: string[] (e.g., ["AI students", "Robotics students", "Software engineers"])
  - prerequisites: string[] (e.g., ["ROS 2 knowledge", "Simulation experience"])

### 2. Chapter
- **Description**: Individual chapters within the VLA module
- **Fields**:
  - id: string (unique identifier for the chapter)
  - title: string (chapter title)
  - moduleRef: string (reference to parent module)
  - objectives: string[] (learning objectives for the chapter)
  - contentPath: string (path to the markdown content)

### 3. VoiceCommand
- **Description**: Educational examples of voice commands and their processing
- **Fields**:
  - id: string (unique identifier)
  - originalText: string (the raw voice command)
  - processedText: string (text after speech recognition)
  - structuredInput: object (structured representation of the command)
  - expectedAction: string (what the command should do)

### 4. ActionPlan
- **Description**: Educational examples of LLM-generated action plans
- **Fields**:
  - id: string (unique identifier)
  - naturalLanguageTask: string (the original natural language input)
  - actionSequence: object[] (sequence of executable actions)
  - ros2Mapping: object (mapping to ROS 2 behaviors)
  - validationStatus: string (valid/invalid/needs review)

### 5. VLAWorkflow
- **Description**: End-to-end workflow examples for educational purposes
- **Fields**:
  - id: string (unique identifier)
  - name: string (workflow name)
  - description: string (overview of the workflow)
  - components: string[] (VLA components involved)
  - steps: object[] (sequence of steps in the workflow)
  - simulationScenario: string (description of the simulation environment)

## Relationships
- Module contains many Chapters
- Chapter may reference many VoiceCommand examples
- Chapter may reference many ActionPlan examples
- ActionPlan connects to ROS 2 behaviors
- VLAWorkflow integrates all components

## Validation Rules
- Each chapter must have at least one learning objective
- Voice commands must have both original and processed text
- Action plans must be mappable to ROS 2 behaviors
- Workflows must include a simulation scenario description
- All content must be appropriate for the target audience level

## State Transitions (for educational examples)
- Draft → Review → Published (content workflow)
- Invalid → Corrected → Valid (action plan validation)