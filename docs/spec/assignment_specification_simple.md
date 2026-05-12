# Programming Autonomous Robots — Assignment Specification (Simplified)

**Course:** COSC2781 (PG) / COSC2814 (UG) | Semester 1 2026
**Weight:** 50% of final course mark
**Submission:** Online via Canvas

---

## 1. Overview

You research and implement a solution to a problem in autonomous robotics on a physical robot platform (AIIL or VXLab robots). You conduct a live demonstration and write a report.

Key requirements:
- Implement at least one solution to the problem
- Demonstrate on a **physical robot** (simulation can help development but final eval must be real)
- Show depth in core autonomous robotics topics from the course
- Research state-of-the-art approaches and adapt them
- Present a live demo + report with research, analysis, and evaluation

### UG vs PG Differences

| Aspect | UG | PG |
|---|---|---|
| **Focus** | Depth and breadth of technical capability — implementing ROS2 infrastructure, robot software architecture, multiple algorithms | Experimental design and evaluation — design experiments suitable for a published paper |
| **Key requirement** | Show broad technical capability across the autonomous software stack | Prove quality of work through rigorous, repeatable experiments |

---

## 2. Group Work

- Groups of 3-4 students (mixed UG/PG allowed)
- Individual contributions are **separately graded** — document who does what
- Use group work tools: MS Teams, Trello, Git
- Work closely together so everyone understands the whole system

---

## 3. AI Tools Policy

- **Allowed for:** ROS2 boilerplate code, ideas, planning, drafting, debugging
- **NOT allowed for:** producing your core work without attribution
- **You MUST submit a log of AI tool usage** — export your chat history
- Suspected use without a log may result in grade penalties

---

## 4. Assessment Requirements

### 4.1 Investigation & Research
- Research algorithms/techniques related to your project
- Cite all relevant research in your report

### 4.2 Original Implementation
- At least one significant element must be your own original implementation
- Cannot be entirely off-the-shelf ROS2 packages
- May significantly modify/adapt existing techniques

### 4.3 Robot Software Architecture
- Choose and justify an appropriate architecture (SPA, Subsumption, Three-tier, etc.)
- Show your project adheres to this architecture
- **Exception for RoboCup Soccer:** The RedBackBots codebase has a fixed architecture that cannot be modified

### 4.4 Autonomy
- Software must be fully autonomous unless scoped otherwise

### 4.5 Open-ended Exploration
- Projects are open-ended — more complex solutions = higher grades
- Synthesis knowledge across the course

### 4.6 Demonstration & Analysis
- Live demo of your autonomous software
- Analysis in your report highlighting strengths AND limitations
- Collect evidence to support capability claims

### 4.7 Report
- Max **10 pages** (including figures, tables, references)
- Single column, ≥1.5cm margins, ≥11pt font
- Structure like a research paper: Introduction, Related Work, Methodology, Results, Analysis, Conclusion

### 4.8 Activity Risk Assessment (ARA)
- Must be completed and **approved by course coordinator** before any robot work
- Template on Canvas

---

## 5. UG-Specific Requirements

### 5.1 ROS2 Infrastructure & Robot Software Architecture
- Some platforms need additional infrastructure setup
- Implement data structures and ROS2 nodes that enforce your chosen architecture
- **RoboCup Soccer exception:** Architecture is fixed — UG students focus on multiple algorithm implementations instead

### 5.2 Extended/Additional Algorithm Implementation
- Implement multiple solutions if infrastructure work isn't sufficiently significant
- Discuss scope with course coordinator

---

## 6. PG-Specific Requirements

### 6.1 Experimental Design and Evaluation
- Design an experiment that explicitly highlights **both strengths AND limitations**
- More than just recording the robot — collect **repeatable statistical measures**
- Standard should be sufficient for a **published article**
- Propose experiment methodology during **Week 12 progress update**
- Plan timeline to allow time for experiments after implementation

### 6.2 Grading in Mixed Groups
- Each student graded per their level's rubric
- Shared components get same grade for all members
- Level-specific components graded separately per level

---

## 7. Projects Available

| # | Project | Robot |
|---|---|---|
| 4.1 | Human Safe Animal behaviours | Unitree Go2 |
| **4.2** | **RoboCup Soccer** | **Booster K1** |
| 4.3 | RoboCup@Home HRI tasks | Tiago |
| 4.4 | AR for Explainable AI | ROSBot Pro |
| 4.5 | Deep Reinforcement Learning | Various |
| 4.6 | LLM Conversational Robot | Nao / Booster K1 / GR1-Pro |
| 4.7 | Macadamia Farm Navigation | ROSBot / Panther |
| 4.8 | Pick-and-Place with Cobot Arm | UR5e / xArm6 |
| 4.9 | Multi-ROSBot Game | Multiple ROSBots |
| 4.10 | Person Recognition & Tracking | Panther |
| 4.11 | ROSBot Package Pickup & Delivery | ROSBot |
| 4.12 | Negotiated Project | Your choice |

---

## 8. RoboCup Soccer — Project Details (Section 4.2)

**Goal:** Enable the Booster K1 robot to compete in RoboCup HSL 2026.

**Available scope components (negotiate with coordinator):**
1. **Localisation** on the soccer field ← *Your group's focus*
2. Robot vision and detection (field features, ball, other robots)
3. Motion for reliable movement and kicking

**Key constraints:**
- Must use existing **RedBackBots RoboCup codebase**
- The codebase has a **defined robot software architecture that cannot be modified** (to avoid interfering with July 2026 competition prep)
- UG students must explore **multiple solutions** instead of infrastructure/architecture work
- Final demo must be on the **Booster K1 under competition conditions**
- PG student must design experiment for competition conditions

---

## 9. Demonstrations

### 9.1 Week 12 Progress Update (20%)
Show:
- Project is on-track per scope
- Components for a minimally viable solution
- Timeline for completion
- Plan for improvements beyond MVP
- **(UG)** Plan for extended infrastructure/implementation
- **(PG)** Proposed experimental evaluation

Submit on Canvas:
- Current software copy
- Completion plan
- Evidence of individual work

### 9.2 Final Demonstration (40%) — Week 15
- Live demo showing project meets requirements
- Distinguish your original work from existing software/literature
- Showcase capabilities
- **(UG)** Show extended infrastructure/architecture/implementation
- **(PG)** Show experimental evaluation results
- Be well-prepared — minimal setup time lost

---

## 10. Report Structure

Max 10 pages. Suggested sections:

1. **Introduction** — Problem statement, scope
2. **Related Work** — Existing methods, literature review
3. **Methodology** — Your approach, implementation, architecture
4. **Results** — What your system does, evidence
5. **Analysis & Evaluation** — Strengths, limitations, (PG) experimental results
6. **Conclusion** — Summary, future work
7. **References** — Full citations of all sources

Include:
- Data tables and figures (legible at 100% zoom and in B&W)
- Log of AI tool usage
- Record of individual contributions

---

## 11. Marking Breakdown

| Component | Weight |
|---|---|
| **Progress Update** — Demonstration | 10/50 (20%) |
| **Final Demo** — Implementation | 10/50 (20%) |
| **Final Demo** — Evaluation & Results | 5/50 (10%) |
| **Final Demo** — Individual Contribution | 5/50 (10%) |
| **Report** — Methodology | 5/50 (10%) |
| **Report** — Analysis & Evaluation | 10/50 (20%) |
| **Report** — Writing & Referencing | 5/50 (10%) |

---

## 12. Key Rules

- **Late submission:** 10% per day penalty, capped at 5 business days (then zero)
- **Extensions:** Only via Special Consideration
- **Plagiarism:** Zero marks for the assignment or course failure
- **Help after Week 12:** Limited — book appointments in advance
