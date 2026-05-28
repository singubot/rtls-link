---
name: feature-dev
description: 'Discuss and align first, then implement in a worktree and open a draft PR of the new feature'
---

Execute this workflow in two mandatory phases.

Do not rush to implementation. Phase 1 must complete first, with explicit user confirmation, before Phase 2 starts.

## Inputs
- Feature title: `$ARGUMENTS`
- If the title is empty, ask the user for a short feature title before continuing.

## Phase 1: Discussion and Alignment (Mandatory)
1. Understand intent fully before proposing implementation details or asking for clarifications.
2. Explore the relevant codebase areas and everything related to the task.
3. Ask targeted clarification questions until ambiguity is removed. Some relevant areas are:
   - Exact expected behavior and acceptance criteria
   - Edge cases and non-goals
   - Constraints (compatibility, performance, security, rollout)
4. Discuss high-level implementation options with tradeoffs:
   - What options do we have? Which one do you recommend and why?
   - Is a refactor-first approach beneficial here?
5. Align with the user on testing scope:
   - Unit test coverage extent
   - Integration test needs
   - Manual and explicit testing/validations ( ej, connecting to remote environments/machine and test on actual hw, debug communications and running temporary helper/test scripts for further validation during this process)
   - What can be skipped and under which conditions
6. Produce an Implementation Agreement summary.
7. Ask for explicit go-ahead.
8. Do not start Phase 2 until the user confirms explicitly.
NOTE: During the discussion phase, the user might ask you ( you can suggest him also ) to write down your investigations, analisys, reports, questions and clarifications to a markdown file inside `.agents/docs/<tile>.md`. This is interesting in order to keep records and important findings encountered during this discussion/planification phase. Feel free also to suggest to the user to write relevant documentation down if you see a clear benefit to doing so.
IMPORTANT: If an Implementation Agreement is written down, keep it as a local working note only. Do not commit it, push it, link it, or list it as PR content unless the user explicitly asks for a permanent documentation artifact.
IMPORTANT: Files created under `.agents/docs/` are local working notes only. Leave them untracked and never commit or push them upstream. If one is accidentally staged, remove it from the commit before proceeding.
IMPORTANT: Do not place temporary investigation/report notes under tracked documentation paths (for example `docs/investigations/`) unless the user explicitly asks for a permanent documentation artifact to be committed.
IMPORTANT: If temporary investigation files are found in tracked paths, keep them completely unstaged before committing and pushing.

## Phase 2: Execution Workflow (After Approval)
1. Derive a branch slug from the title: lowercase kebab-case, only `[a-z0-9-]`, collapse repeated dashes.
2. Branch name format is mandatory: `feature/<slug>`.
3. Identify the git repository root.
4. Fetch latest refs and prune stale ones: `git fetch --all --prune`.
5. Select the base branch in this order, using remote first:
   - `develop`
   - `main`
   - `master`
6. If none of those branches exist locally or on `origin`, stop and report the issue.
7. Ensure the selected base is up to date by starting from `origin/<base-branch>`.
8. Create a new worktree and feature branch from that up-to-date base:
   - Default worktree path: `../wt-<repo-name>-<slug>`
   - If that path already exists, append a timestamp suffix.
   - Example command shape: `git worktree add -b "feature/<slug>" "<worktree-path>" "origin/<base-branch>"`
9. Perform all implementation work only inside the new worktree.
10. Implement the feature according to the agreed plan from Phase 1.

## Validation
1. Attempt to compile/build if the project supports it.
2. Add or update unit tests according to the agreed testing scope.
3. Run targeted unit tests first.
4. Run integration tests when agreed and feasible.
5. Execute the agreed manual validation. Skip if not applicable.
6. If compilation/unit/integration/manual validation is not possible or would require disproportionate setup/work, explicitly document what was skipped and why, then continue.

## Review
Perform an in-depth review. Evaluate as well if there is any "dead code" left or code that during this changes have become unuseful and should be removed/simplified. The review should be constructive, adding suggestions and guidance to each of the findings.

Once the review is complete, the findings should be fixed and the validation rerunned. The validation should attempt to include targeted testing where possible so we have a high confidence to have covered the findings and not broken anything during the process.

## Phase 3:
If phase 2 has gone according to plan, the tests are passing, manual verification is healthy (if applicable) and there are no critical or high severity issues left, continue to phase 3 without asking the user.

## Commit, Push, and Draft PR
1. Stage only relevant files. Never stage local-only investigation/report notes (for example `.agents/docs/*` and temporary files under `docs/investigations/*`).
2. Create a clear commit message describing the feature.
3. Push with upstream: `git push -u origin "feature/<slug>"`.
4. Select PR base branch in this order (remote): `develop`, fallback `main`, fallback `master`.
5. Open a draft PR against that base branch and assign the current GitHub user as assignee.
   - Prefer creating it with `--assignee @me`.
   - If the CLI flow requires a separate step, immediately run `gh pr edit --add-assignee @me` after creation.
6. Before creating or finalizing the PR, check the available repo labels with `gh label list` and attach the appropriate ones.
7. PR body must include:
   - Intent of the PR
   - Feature description
   - Validation summary (build/unit/integration results, plus skipped checks and reasons)
   - Reviewer context (key decisions, refactor rationale if any, risks, migration/deployment notes, follow-ups)
8. End the PR body with a footer in this format:
   - `Generated by <model name>`

## Final Output to User
Report:
- Base branch used
- Worktree path
- Feature branch name
- Validation executed and skipped (with reasons)
- Commit SHA
- Draft PR URL
- Any relevant issues or relevant notes that the user should be aware of

## Guardrails
- Keep the primary workspace untouched; do not implement in the original checkout.
- Do not force-push.
- If a command fails, diagnose and retry with the smallest safe fix.
- During Phase 1, proactively ask clarifying questions instead of inferring critical product decisions.

# NOTE
If, during these command steps, you encounter errors, blockers, or generic issues (such as strange behavior from the GitHub CLI or other relevant tools in this workflow), create a markdown file in `.agents/blockers` folder. Document the issues encountered and any information that would have been helpful to know from the start, for example, the correct way to execute a specific `gh` command, unexpected helper script beheivours, etc.
