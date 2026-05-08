# New Issues — ARA Completion, Issue Extractor Testing, and Project Scope Finalisation

> Three new issues derived from [`docs/new_issues_drafts.md`](../docs/new_issues_drafts.md).
> Each issue includes labels, task description, and acceptance criteria.
> Labels with a `**` are new labels that need to be created in the repository.

---

## ARA-01: Complete the Activity Risk Assessment for HTC Vive Motion Tracking

**Labels:** `documentation` `safety` `**` `ara` `**` `htc-vive` `**`

**Assignee:** Thomas Gosling (@wholemealbaby on GitHub)

**Priority:** P0 — blocks all lab-based development activities

**Milestone:** Planning Phase (Week 9–11)

### Task Description

Complete the Activity Risk Assessment (ARA) for motion capture using HTC Vive equipment. The ARA is a prerequisite before any development activities or use of the robot may commence, as specified in the assignment spec ([`docs/assignment_spec.txt`](../docs/assignment_spec.txt:136-138)). The ARA must be approved by the course coordinator before work can begin.

A draft ARA has been started at [`docs/ara/ara_htc_vive_motion_tracking.md`](../docs/ara/ara_htc_vive_motion_tracking.md) covering six activities:

1. **Activity 1:** Setting up HTC Vive Base Stations on Tripods
2. **Activity 2:** Setting up HTC Vive Tracker and Dongle
3. **Activity 3:** Charging HTC Vive Trackers
4. **Activity 4:** Operating the HTC Vive Motion Tracking System (Vive ROS2 Integration)
5. **Activity 5:** Updating Firmware (Base Stations and Trackers)
6. **Activity 6:** Packing Up and Storing HTC Vive Equipment

Each activity has been populated with hazards, risks, current controls, risk ratings, and additional control measures. The remaining work involves:

- [ ] Reviewing each activity for completeness and accuracy
- [ ] Confirming the risk ratings (Consequence × Likelihood) are correctly assigned per the Booster P&ERA matrix
- [ ] Ensuring all additional risk control measures are feasible and assigned to a responsible person
- [ ] Getting the ARA signed off by the course coordinator
- [ ] Storing the final approved version in the repository

### Acceptance Criteria

- [ ] All six activities in [`docs/ara/ara_htc_vive_motion_tracking.md`](../docs/ara/ara_htc_vive_motion_tracking.md) are reviewed and complete
- [ ] Risk ratings (Consequence, Likelihood, Risk Rating) are correctly assigned for both "Current" and "Residual" columns
- [ ] All additional risk control measures are practical, specific, and assigned to a named responsible person
- [ ] The ARA has been submitted to and approved by the course coordinator
- [ ] The approved ARA is committed to the repository at `docs/ara/ara_htc_vive_motion_tracking.md`
- [ ] Evidence of approval (email, signed form, or Canvas submission) is linked from the ARA document

### Dependencies

- Access to the Booster P&ERA: AIIL-Booster-K1-PAR-Development risk matrix for verifying risk ratings
- Course coordinator availability for ARA review and approval

---

## Test Issue Extractor for Transfer of Issues from RedBackBots Soccer Repository

**Labels:** `infrastructure` `automation` `**` `testing` `**`

**Assignee:** TBD

**Priority:** P1 — needed before the team can efficiently track work across repositories

**Milestone:** Planning Phase (Week 9–11)

### Task Description

Test the issue extractor script that transfers issues from the RedBackBots Soccer GitHub repository to our assignment repository and project board. This involves two phases:

**Phase 1 — Script Functionality Test:**
Verify the issue extractor script works correctly in isolation. This includes confirming it can authenticate with the GitHub API, parse issues from the source repository, and format them correctly for the target repository.

**Phase 2 — Automation Test:**
Test the full automated pipeline, including:
- Triggering the extraction (manual or scheduled)
- Creating issues in the target repository with correct labels, assignees, and formatting
- Preserving issue metadata (comments, labels, milestone associations where applicable)
- Handling edge cases (duplicate issues, closed issues, issues with missing fields)

### Background

The RedBackBots RoboCup codebase ([`docs/assignment_spec.txt`](../docs/assignment_spec.txt:249-252)) already contains a defined robot software architecture and existing issues. To avoid manual duplication and ensure traceability, we need an automated mechanism to transfer relevant issues from the RedBackBots repository to our assignment repository. This also helps distinguish between pre-existing work and our original implementation.

### Tasks

- [ ] **Phase 1:** Run the issue extractor script against a test set of issues from the RedBackBots repository
- [ ] Verify the script authenticates successfully with the GitHub API
- [ ] Verify the script correctly parses issue titles, descriptions, labels, and comments
- [ ] Verify the output format matches the target repository's issue template
- [ ] **Phase 2:** Set up the automation trigger (GitHub Actions, cron job, or manual workflow)
- [ ] Run a full end-to-end test: extract a batch of issues and confirm they appear correctly in the target repository
- [ ] Test edge cases: duplicate detection, label mapping, closed vs open issues
- [ ] Document the extraction process and automation setup

### Acceptance Criteria

- [ ] **Phase 1:** Script runs without errors against a test set of ≥ 5 issues from the RedBackBots repository
- [ ] **Phase 1:** Output issues contain correct title, body, labels, and metadata matching the source
- [ ] **Phase 2:** Automation pipeline runs successfully end-to-end at least once
- [ ] **Phase 2:** Transferred issues appear in the target repository with correct formatting
- [ ] **Phase 2:** No duplicate issues are created (idempotent execution)
- [ ] Edge cases handled: issues with missing labels, closed issues, issues with long descriptions
- [ ] Extraction process documented in `docs/issue_extraction_guide.md` or similar

### Dependencies

- Access to the RedBackBots GitHub repository (read permissions)
- GitHub API token or authentication mechanism configured
- Issue extractor script available (may need to be created if not already existing)

---

## Finalize Project Scope; Natural Landmark Feature Extraction & Particle Filter

**Labels:** `planning` `research` `requirements` `architecture` `**`

**Assignee:** Team Lead / All team members

**Priority:** P0 — blocks all implementation work

**Milestone:** Planning Phase (Week 9–11)

### Task Description

Finalize the project scope following lecturer feedback that the existing Kalman filter with offline optimization approach is not acceptable. The lecturer identified two issues:

1. **Autonomy requirement:** Offline optimization does not satisfy the autonomous aspect of the assignment. Code must execute on the robot itself (except for the external pose estimation experiment).
2. **Originality requirement:** The existing filter was implemented by the RedBackBots Soccer team, creating too much overlap — making it hard to distinguish what was implemented for the assignment vs what was pre-existing.

**New Direction:**
The lecturer instructed the team to focus on:
- **Natural landmark feature extraction** to break symmetry in the robot's localisation
- Implement a **new particle filter** that can use the landmark feature extraction as a prior

### Tasks

- [ ] **Team discussion:** Confirm the new approach (landmark feature extraction + particle filter) with the full team
- [ ] **Alternative consideration:** Discuss whether the team prefers to pursue a different aspect (e.g., vision) instead
- [ ] **UG infrastructure requirement:** Review [`docs/assignment_spec.txt`](../docs/assignment_spec.txt:150-157) to determine if the external pose estimation experiment satisfies the infrastructure/undergrad requirement. The team has three undergrads and one postgrad — assess whether:
  - The infrastructure requirement is sufficient for all three undergrads
  - The repeat implementation requirement (Section 3.2, lines 158-162) needs to be satisfied for some students
- [ ] **Document scope:** Write up the final agreed scope including:
  - Which component(s) of the RoboCup HSL rules will be addressed
  - The algorithms to be implemented (landmark extraction, particle filter)
  - The external pose estimation experiment scope
  - How UG vs PG requirements are distributed across team members
- [ ] **Get coordinator approval:** Present the finalized scope to the course coordinator for sign-off by the Week 9 Thursday workshop deadline

### Background

The assignment spec ([`docs/assignment_spec.txt`](../docs/assignment_spec.txt:240-254)) describes the RoboCup Soccer project. The existing RedBackBots codebase already contains a Kalman filter for localisation. Per the spec (lines 249-252), the existing robot software architecture is not permitted to be modified to prevent interference with RoboCup competition preparations. Therefore, the new particle filter and landmark extraction must be implemented as a separate, complementary system.

The external pose estimation experiment (using HTC Vive motion tracking, see [`docs/ara/ara_htc_vive_motion_tracking.md`](../docs/ara/ara_htc_vive_motion_tracking.md)) provides a ground-truth reference for evaluating the particle filter's accuracy. This may satisfy the UG infrastructure requirement (Section 3.1), but this needs to be confirmed.

### Acceptance Criteria

- [ ] Team has discussed and agreed on the new approach (landmark extraction + particle filter) or an alternative
- [ ] The chosen approach is documented in a scope document committed to the repository
- [ ] The scope document addresses:
  - Which RoboCup HSL component(s) are in scope
  - The algorithms to be implemented (landmark extraction, particle filter)
  - The role of the external pose estimation experiment
  - How UG infrastructure requirements are satisfied for all three undergrads
  - Whether the repeat implementation requirement applies to any UG students
  - How PG experimental evaluation will be designed
- [ ] Scope has been presented to and approved by the course coordinator
- [ ] Approval is documented (email, meeting notes, or signed form) and linked from the scope document
- [ ] Scope is finalized by the Week 9 Thursday workshop deadline

### Dependencies

- Course coordinator availability for scope discussion and approval
- Team meeting to discuss and agree on the approach
- Review of [`docs/assignment_spec.txt`](../docs/assignment_spec.txt) Section 3 (lines 147-209) for UG/PG requirements

---

## New Labels Required

The following new labels should be created in the GitHub repository with the suggested colours:

| Label | Colour | Description |
|-------|--------|-------------|
| `safety` | `#B60205` (bold red) | Safety-related issues, risk assessments, and hazard controls |
| `ara` | `#D93F0B` (bold orange) | Activity Risk Assessment tasks and documentation |
| `htc-vive` | `#1D76DB` (bold blue) | HTC Vive equipment setup, operation, and integration |
| `automation` | `#0E8A16` (bold green) | Automation scripts, CI/CD, and workflow tooling |
| `testing` | `#BFD4D9` (bold grey) | Testing tasks, test scripts, and verification |
| `architecture` | `#5319E7` (bold purple) | Software architecture decisions and design |
