# RoboCup Soccer — Project TL;DR & Gotchas

**Course:** COSC2781 (PG) / COSC2814 (UG) — Semester 1 2026
**Group:** 3 × UG + 1 × PG
**Robot:** Booster K1
**Project Scope:** Localisation with a new particle filter + natural landmark feature selection for breaking symmetry on the field using depth and vision.

---

## TL;DR — What You Actually Need to Do

### For Everyone (All 4 Members)

| What | When | Weight |
|---|---|---|
| **Week 12 Progress Update** — Live demo of current progress, show you're on-track, present plan for completion | Week 12 workshops | 20% |
| **Final Demonstration** — Live demo of working solution on the Booster K1 under competition conditions | Week 15 (by appointment) | 40% |
| **Final Report** — Max 10 pages, structured like a research paper (Intro, Related Work, Methodology, Results, Analysis, Conclusion) | Sun 21 Jun 2026, 23:59 | 40% |
| **Activity Risk Assessment (ARA)** — Must be approved *before* you can start working with the robot | ASAP | 2% |

### UG Members (3 Students) — Specific Expectations

1. **Extended/Additional Algorithm Implementation** — Since the RedBackBots codebase already has a defined robot software architecture that you **cannot modify**, UG students must explore **multiple solutions** to the localisation problem. This means:
   - Implement your new particle filter
   - Also implement the natural landmark feature selection
   - These count as the "multiple solutions" / "extended implementation" requirement

2. **ROS2 Infrastructure & Robot Software Architecture** — The rubric mentions UG students implement infrastructure. However, since the RedBackBots architecture is fixed, your "infrastructure" contribution is the new particle filter integration and landmark selection system that plugs into the existing architecture.

### PG Member (1 Student) — Specific Expectations

1. **Experimental Design & Evaluation** — You must design an experiment that would be publishable-quality. This means:
   - Not just "robot drives and localises"
   - Repeatable, statistical measures of localisation accuracy
   - Compare your new particle filter + landmark selection against the baseline (existing RedBackBots localisation)
   - Show *strengths* AND *limitations* — e.g., where does symmetry-breaking fail? Under what conditions?
   - Propose your experiment methodology during the Week 12 progress update

2. **Extended Evaluation Experiment and Analysis** — Go beyond the standard analysis that everyone does.

---

## Key Dates & Deadlines

| Date | Milestone |
|---|---|
| **Thu, Week 9** | Project scope finalised with course coordinator |
| **Week 12 workshops** | Progress update demo + submit current code + plan |
| **Week 15 (by appointment)** | Final live demonstration |
| **Sun 21 Jun 2026, 23:59** | Report due |
| **ASAP** | ARA must be approved before touching the robot |

---

## 🚨 Gotchas & Considerations

### 1. RedBackBots Codebase — You CANNOT Modify the Architecture

> *"This codebase already contains a defined robot software architecture, which is not permitted to be modified to prevent interference with preparations for the RoboCup competition held in July 2026."*

This is a **hard constraint**. Your particle filter and landmark selection must be designed as **pluggable modules** that work within the existing architecture. You cannot refactor or restructure the existing code.

**Implication:** UG students can't demonstrate "infrastructure/architecture" in the traditional sense. The rubric says UG students must show extended infrastructure — but the spec also says for RoboCup Soccer, UG students will instead explore **multiple solutions**. Make sure the course coordinator is clear on this.

### 2. Symmetry Breaking on the Field — The Core Challenge

The RoboCup HSL field is symmetric (green field, white lines, goals at each end). Without distinguishing features, a particle filter can converge on the wrong half of the field (the "kidnapped robot problem" in reverse).

Your approach — **natural landmark feature selection using depth and vision** — is the key innovation. Consider:

- **What natural landmarks exist?** Field lines, goal posts, corner arcs, centre circle. The HSL rules define the field layout precisely.
- **Depth information** from the Booster K1's camera can disambiguate distance-to-goal, which is critical.
- **Vision features** — line intersections (T-junctions, L-corners, X-crossings) are unique per field quadrant.
- **Failure modes:** If the robot can't see any landmarks (e.g., facing away from everything), the filter may still be ambiguous. Have a recovery strategy.

### 3. Competition Conditions for Final Demo

> *"You must demonstrate your final code on the Booster K1 robot under competition conditions."*

This means:
- The field will be set up per HSL 2026 rules
- Lighting may not be ideal (lab lighting ≠ competition lighting)
- You need to handle the robot being moved (kidnapped) during operation
- The PG student's experiment must be designed for these conditions

### 4. PG Experiment Design — Start Early

The PG student needs to:
- Propose experiment methodology by **Week 12**
- This means implementation needs to be mostly done **before** Week 12
- Experiments then run Weeks 13-14
- **Risk:** If implementation slips, there's no time for proper experiments

**Recommendation:** Get the particle filter working in simulation first. Use TheConstruct or a local ROS2 setup. Then port to the real robot.

### 5. Limited Help After Week 12

> *"After Week 12, there are no formally scheduled classes. Limited help will be available... any in-person help will need to be conducted by a pre-arranged appointment."*

**You need to be self-sufficient from Week 13 onwards.** Make sure:
- All hardware access is sorted before Week 12
- You have a clear plan for Weeks 13-14
- You've asked all critical questions during the Week 12 progress update

### 6. Booster K1 Specifics

- Uses ROS2 (likely Humble, as mentioned for research platforms)
- Camera provides depth + RGB — you'll need both for landmark selection
- The existing RedBackBots codebase has localisation — you're replacing/improving it
- You'll need to coordinate with the RoboCup team to avoid interfering with their competition prep

### 7. AI Tools Log

> *"You must submit a log of your use of generative AI tools during this assessment."*

Every member needs to track their AI tool usage. Export logs from ChatGPT, Claude, Gemini, etc. **Suspected use without a log may result in grade penalties.**

### 8. Report Page Limit — 10 Pages Max

> *"No more than 10 pages, including figures, tables and references."*

This is tight for a project with 4 people. Be concise. Figures should be dense with information. Use the appendix sparingly (if at all — the spec doesn't mention one).

### 9. Individual Contributions Are Graded Separately

The rubric has an **Individual Contributions** component (10% of the 40% demonstration mark). You need to document who did what. If there are group work issues, individual grades may be allocated.

**Recommendation:** Use a task tracker (Trello, GitHub Projects) and commit history to evidence每个人的工作.

### 10. UG vs PG Grading in the Same Group

> *"Each student will be graded according to the rubric for their level of study."*

- All 4 members share the same grade for **overlapping components** (demonstration, report sections that apply to everyone)
- UG students are graded on the UG rubric (Extended Infrastructure, multiple solutions)
- PG student is graded on the PG rubric (Experimental Design, Evaluation)
- The PG student's experiment should evaluate the **whole team's work**, not just their own

---

## Suggested Timeline

| Week | Focus |
|---|---|
| **Week 9** | Finalise scope with coordinator. Get ARA approved. Set up development environment. Study RedBackBots codebase. |
| **Week 10** | Implement baseline particle filter. Get it running in simulation. |
| **Week 11** | Implement natural landmark feature selection. Integrate with particle filter. Test on real robot. |
| **Week 12** | **Progress update.** PG student presents experiment plan. Have minimally viable solution working. |
| **Week 13** | Polish implementation. PG student runs formal experiments. Collect data. |
| **Week 14** | Final demo prep. Write report. |
| **Week 15** | **Final demonstration** (by appointment). Report due Sunday. |
