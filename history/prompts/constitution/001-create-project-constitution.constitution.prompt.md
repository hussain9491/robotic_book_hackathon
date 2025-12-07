---
id: 001
title: Create Project Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Claude Code
command: /sp.constitution
labels: [constitution, project-setup, physical-ai, humanoid-robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

@.claude\commands\sp.constitution.md  /sp.constitution

Project: Unified Book + RAG Chatbot for "Physical AI & Humanoid Robotics"

Core principles:
- Technical accuracy across robotics, AI systems, ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA architectures
- Fidelity to authoritative primary sources (official docs: ROS, Gazebo, Unity, Isaac, OpenAI Agents, Whisper)
- Clarity for an engineering audience (upper-undergraduate / graduate level)
- Modular consistency across book chapters, code examples, and RAG knowledge chunks
- Maintainability: All content structured for Docusaurus + GitHub Pages publishing
- RAG integrity: Chatbot responses strictly grounded in book text or selected user snippets

Key standards:
- All robotics concepts must be validated through primary technical documentation
- All code examples must be executable and tested (Python, ROS 2 rclpy, FastAPI, Agents SDK)
- All diagrams must follow a consistent system-architecture style (mermaid + PNG exports)
- RAG chunks must be atomic, self-contained, and context-safe
- Knowledge sources:
    -- ROS 2 (Humble/Iron) official documentation
    -- Gazebo Garden/Ignition official docs
    -- Unity robotics documentation
    -- NVIDIA Isaac Sim + Isaac ROS documentation
    -- OpenAI Agents/ChatKit SDK documentation
    -- Neon Serverless Postgres + Qdrant documentation

Format & structure:
- Book framework: Docusaurus v3
- Deployment: GitHub Pages
- Code integration: Claude Code + Spec-Kit Plus atomic workflows
- RAG backend:
    -- FastAPI inference server
    -- Neon Serverless Postgres for chat logs & metadata
    -- Qdrant Cloud Free Tier for vector embeddings
    -- OpenAI Agents/ChatKit SDK for orchestration
- File structure must follow Docusaurus best practices:
    -- /docs
    -- /src/components
    -- /rag/api
    -- /rag/db
    -- /rag/vectorstore
    -- /spec (Spec-Kit generated artifacts)

Constraints:
- Textbook word count: 40,000–55,000 words
- Every chapter must include:
    -- Learning objectives
    -- Architecture diagrams
    -- ROS/Gazebo/Isaac code examples
    -- Implementation labs
    -- Review questions
- RAG chatbot:
    -- Must answer only from book text or selected snippet
    -- Must reject questions outside book scope
- All source claims must cite an official documentation URL (non-APA)
- Zero hallucinations allowed in RAG mode
- Zero proprietary code unless open-sourced or licensed

Success criteria:
- Book compiles and publishes successfully on GitHub Pages using Docusaurus
- RAG chatbot embedded and functioning inside the book site
- Chatbot correctly answers:
    -- global book questions
    -- snippet-based questions
- All ROS 2, Gazebo, Unity, Isaac, and Agents SDK code validated through dry-run or simulation
- All Spec-Kit Plus tasks produce deterministic outputs
- Book passes internal technical audit:
    -- correctness
    -- clarity
    -- reproducibility
- Deployment pipeline: GitHub Actions automation fully verified

## Response snapshot

Created a comprehensive constitution file for the "Physical AI & Humanoid Robotics" book project. The constitution includes:

- Core Principles: Technical accuracy, fidelity to primary sources, engineering audience clarity, modular consistency, maintainability, and RAG integrity
- Technical Standards: Source validation requirements, knowledge sources, and format/structure requirements
- Development Workflow: Content creation standards and quality assurance measures
- Governance: Clear guidelines for constitutional compliance and amendments

The constitution file (.specify/memory/constitution.md) has been updated with all the specific requirements and principles for the project, replacing the template placeholders with concrete project values.

## Outcome

- ✅ Impact: Created initial project constitution for Physical AI & Humanoid Robotics book
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Project constitution established for future development work
- 🧠 Reflection: Constitution provides clear guidelines for technical accuracy and content validation

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing specific book chapters following constitutional guidelines