---
name: audit-loop
description: Focused audit loop for finding real blockers, meaningful friction, implementation risks, and agreement mismatches while avoiding nitpicks.
---

# Audit Loop

Use this skill to audit a plan, implementation agreement, or proposed workflow before execution.

## Process
1. Restate the user intent and the current agreement in a few lines.
2. Inspect the relevant codebase context if the audit depends on implementation details.
3. Run 2 to 5 focused audit passes depending on complexity. Use sub-agents only when the user or active workflow explicitly authorizes multi-agent work.
4. Ask each pass to look for material issues only:
   - blockers
   - risky assumptions
   - missing requirements
   - integration or compatibility issues
   - test or validation gaps
   - unnecessary complexity
   - mismatches between user intent and the agreement
5. Ignore style nitpicks, speculative preferences, and minor wording suggestions.
6. Merge duplicate findings, discard weak findings, and keep only actionable points.
7. If meaningful findings remain, discuss them with the user and update the agreement.
8. Repeat until there are no meaningful open blockers or questions.

## Output
End with:
- A concise audit summary
- Any agreement changes made during the loop
- Remaining risks, if any
- A clear statement that the plan is ready for explicit execution approval, or what is still blocked
