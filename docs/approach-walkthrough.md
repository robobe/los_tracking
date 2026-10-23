# LOS approach: step-by-step code walkthrough

Start at the navigation entry point, understand each decision, then follow its
called methods into estimation, geometry and control. Each step connects the
guidance idea to the implemented math and the configuration that changes it.

## Menu

1. [Entry method: purpose, input contract, retained state and execution map](walkthrough/entry-method.md)
2. [Update cycle: validation through command expiry](walkthrough/update-cycle.md)
3. [Worked example: cold start to a tracking tick](walkthrough/worked-example.md)
4. [Failure examples and entry-method review notes](walkthrough/failure-and-review.md)
5. [Next chapters: review roadmap](walkthrough/review-roadmap.md)

## Review status and reading guide

**Completed review:** the entry method's contract and orchestration, checked against
current source and focused existing test cases. Called-method internals are
summarized only as needed to explain the entry method. Their detailed reviews
remain pending in the [roadmap](walkthrough/review-roadmap.md).

Companion documents:

- [Package control flow](package-control-flow.md): overall architecture and signal paths.
- [Parameter reference](package-parameters.md): complete configuration defaults, validation limits and value-change effects.

Future chapters use the same structure: purpose, inputs/state, execution order,
math/units, configuration, worked example and failure behavior. Review findings
are recorded separately from explanations of implemented behavior. This document
describes the existing algorithm; it does not change it.

## Verification and limits

The entry chapter was checked against current source, configuration defaults and
the focused test cases linked in the [failure examples](walkthrough/failure-and-review.md). The worked example and document links were
checked separately. Existing test names are evidence of coverage in source, not
a claim that the full test suite was run for this documentation change.

No runtime algorithm, API or configuration was changed. Detailed called-method
reviews remain pending. No flight simulation or hardware validation was performed;
this walkthrough does not establish flight stability or physical control authority.
