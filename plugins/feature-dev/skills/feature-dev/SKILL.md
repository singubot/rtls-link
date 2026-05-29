---
name: feature-dev
description: 'Workflow process for discussion, alignment, implementation and verification of a new feature'
---

Use this workflow in three phases:
- Phase 1 -> Phase 2: when the agent has no meaningful open questions, create the feature worktree, write the agreement there, then move forward.
- Phase 2 -> Phase 3: the user must explicitly approve execution.
- Do not edit product code before Phase 3.
- Agreement files under `.agents/agreements/` are local-only notes. Never stage, commit, push, quote broadly, or copy them into tracked files.

## Phase 1: Discussion and Alignment
1. Understand the user intent, acceptance criteria, non-goals, constraints, and expected testing scope.
2. Inspect the relevant codebase areas before settling on implementation details.
3. Ask targeted clarification questions while meaningful ambiguity remains.
4. Discuss high-level implementation options and recommend one with concise tradeoffs.
5. Derive the branch slug from the feature title: lowercase kebab-case, only `[a-z0-9-]`, collapse repeated dashes.
6. Fetch latest refs and prune stale ones: `git fetch --all --prune`.
7. Identify the repository root, repo name, parent directory, base branch, and base ref. Select the base branch in this order, using remote first:
   - `develop`
   - `main`
   - `master`
8. Use `origin/<base-branch>` as the base ref when available. If only a local branch exists, use `<base-branch>` and report that in the agreement.
9. Write these names in the agreement and in the user-facing Phase 1 summary:
   - Base branch: `<base-branch>`
   - Base ref: `<base-ref>`
   - Branch: `feature/<slug>`
   - Worktree name: `wt_<slug>` unless that path already exists, then `wt_<slug>_YYYYMMDDHHMMSS`
   - Worktree path: `<parent-dir>/.worktrees/<repo-name>/<worktree-name>`
   - Agreement path: `<worktree-path>/.agents/agreements/implementation-agreement.md`
10. Produce a concise Implementation Agreement covering intent, scope, approach, validation, branch, worktree, and agreement path.
11. When there are no more meaningful questions, create the feature worktree before moving to Phase 2:
   - If no allowed base branch exists locally or on `origin`, stop and report the issue.
   - Create the approved worktree and branch from the selected base ref:
     `git worktree add -b "feature/<slug>" "<worktree-path>" "<base-ref>"`
   - If the approved path became occupied, choose a timestamp-suffixed worktree name and update the agreement before continuing.
   - Create `<worktree-path>/.agents/agreements/`.
   - Write the agreement to `<worktree-path>/.agents/agreements/implementation-agreement.md`.
12. Explicitly say Phase 1 is complete and move to Phase 2.

## Phase 2: Audit Agent Loop
Run the audit-loop skill if available. If it is unavailable, perform the audit in the main thread.

Audit the agreement against the user's intent, Phase 1 feedback, and relevant codebase context. Look for missing requirements, blockers, risky assumptions, integration issues, test gaps, rollout concerns, and opportunities to simplify.

Use the agreement file in `<worktree-path>/.agents/agreements/` as the source of truth. Keep updating it during the audit loop whenever scope, approach, risks, validation, or decisions change.

Ask the user only about decisions that materially affect scope or execution. If the audit changes the plan, update the agreement file before asking for approval.

End Phase 2 with a concise audit summary and the final execution agreement path. Ask the user for explicit go-ahead to start Phase 3. Do not proceed to Phase 3 until the user approves.

## Phase 3: Execution Workflow
1. Reuse the approved branch, worktree path, and agreement from Phase 2 unless the user approved a change.
2. If the worktree is missing or no longer matches the approved branch, stop and report the issue.
3. Check for local-only agent context at `<parent-dir>/.worktrees/<repo-name>/.agents` when relevant. Never stage, commit, push, quote broadly, or copy that context into tracked files.
4. Perform all implementation work only inside the feature worktree.
5. Implement according to the final agreement.

## Validation and Review
1. Tests and CI:
   - Run the agreed build, unit, integration, and CI checks first.
   - If a step is impossible or disproportionate, document what was skipped and why.
2. In-depth review:
   - Run the audit-loop skill against the diff, agreement, and original user intent.
   - Look for correctness issues, missed requirements, unnecessary complexity, dead code, test gaps, and plan mismatches.
   - Fix meaningful findings and rerun targeted tests or CI where practical.
3. In-hardware verification:
   - Run actual hardware validation only when specified and feasible.
   - Document skipped hardware checks and reasons.

## Commit, Push, and Draft PR
1. Stage only relevant files. Never stage local-only notes such as `.agents/agreements/*`, `.agents/blockers/*`, `<parent-dir>/.worktrees/<repo-name>/.agents/*`, or temporary investigation files.
2. Create a clear commit message describing the feature.
3. Push with upstream: `git push -u origin "feature/<slug>"`.
4. Open a draft PR against the selected base branch and assign the current GitHub user.
5. Check available repo labels with `gh label list` and apply appropriate labels.
6. PR body must include:
   - Intent
   - Feature description
   - Validation summary, including skipped checks and reasons
   - Reviewer context: key decisions, risks, migration/deployment notes, and follow-ups
7. End the PR body with: `Generated by <model name>`.

## Final Output to User
Report:
- Base branch used
- Worktree path
- Agreement path
- Feature branch name
- Validation executed and skipped
- Commit SHA
- Draft PR URL
- Relevant issues or notes

## Guardrails
- Keep the primary workspace untouched.
- Do not force-push.
- If a command fails, diagnose and retry with the smallest safe fix.
- If workflow or tooling blockers would help future agents, record concise local-only notes in `.agents/blockers/` and never stage them.
