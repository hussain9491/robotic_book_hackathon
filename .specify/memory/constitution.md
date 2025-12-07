<!--
Sync Impact Report:
- Version change: 0.1.0 → 1.0.0 (initial project constitution)
- Modified principles: All principles newly defined for the Physical AI & Humanoid Robotics project
- Added sections: Technical accuracy, Fidelity to primary sources, Clarity for engineering audience, Modular consistency, Maintainability, RAG integrity
- Removed sections: Template placeholders
- Templates requiring updates: ✅ updated
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Technical Accuracy
All robotics, AI systems, ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA architecture concepts must be validated through primary technical documentation and authoritative sources. Content must reflect real-world implementation practices and proven methodologies in the field.

### II. Fidelity to Primary Sources
All content must be grounded in official documentation from ROS, Gazebo, Unity, Isaac, OpenAI Agents, Whisper, and other authoritative technical resources. No secondary interpretations or assumptions should be made without clear citation to primary sources.

### III. Engineering Audience Clarity
Content must be structured for an upper-undergraduate to graduate level engineering audience. Explanations should be technically precise yet accessible, with sufficient depth for implementation while maintaining conceptual clarity.

### IV. Modular Consistency
Book chapters, code examples, and RAG knowledge chunks must maintain consistent terminology, structure, and formatting across all components. Each module should be self-contained while contributing to the overall educational narrative.

### V. Maintainability for Publication
All content must be structured for Docusaurus + GitHub Pages publishing, with clear file organization, proper linking, and adherence to documentation best practices that support long-term maintenance.

### VI. RAG Integrity
Chatbot responses must be strictly grounded in book text or selected user snippets. The system must reject questions outside the book's scope and maintain strict factual accuracy without hallucination.

## Technical Standards

### Source Validation Requirements
- All robotics concepts must be validated through primary technical documentation
- All code examples must be executable and tested (Python, ROS 2 rclpy, FastAPI, Agents SDK)
- All diagrams must follow a consistent system-architecture style (mermaid + PNG exports)
- RAG chunks must be atomic, self-contained, and context-safe

### Knowledge Sources
All content must reference and be validated against:
- ROS 2 (Humble/Iron) official documentation
- Gazebo Garden/Ignition official docs
- Unity robotics documentation
- NVIDIA Isaac Sim + Isaac ROS documentation
- OpenAI Agents/ChatKit SDK documentation
- Neon Serverless Postgres + Qdrant documentation

### Format & Structure Requirements
- Book framework: Docusaurus v3
- Deployment: GitHub Pages
- Code integration: Claude Code + Spec-Kit Plus atomic workflows
- RAG backend: FastAPI inference server with Neon Serverless Postgres for chat logs & metadata, Qdrant Cloud Free Tier for vector embeddings, OpenAI Agents/ChatKit SDK for orchestration
- File structure must follow Docusaurus best practices with organized /docs, /src/components, /rag/api, /rag/db, /rag/vectorstore, and /spec directories

## Development Workflow

### Content Creation Standards
- Textbook word count: 40,000–55,000 words
- Every chapter must include: Learning objectives, Architecture diagrams, ROS/Gazebo/Isaac code examples, Implementation labs, Review questions
- All source claims must cite an official documentation URL (non-APA format)
- Zero hallucinations allowed in RAG mode
- Zero proprietary code unless open-sourced or properly licensed

### Quality Assurance
- RAG chatbot must correctly answer both global book questions and snippet-based questions
- All ROS 2, Gazebo, Unity, Isaac, and Agents SDK code must be validated through dry-run or simulation
- All Spec-Kit Plus tasks must produce deterministic outputs
- Book must pass internal technical audit for correctness, clarity, and reproducibility

## Governance

This constitution governs all development activities for the Physical AI & Humanoid Robotics book project. All contributions must comply with these principles. Amendments require documentation of changes, approval from project maintainers, and a migration plan for existing content. All pull requests and reviews must verify constitutional compliance before merging.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
