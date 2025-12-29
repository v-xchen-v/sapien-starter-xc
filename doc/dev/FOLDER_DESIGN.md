# Repository Folder Design Notes
This repository is designed for **learning first, reuse second**.
This structure intentionally minimizes upfront design and allow structure to emerge naturelly.

```
repo/
├── topics/                 # learning workspaces
├── shared/                 # internal reuse (unstable)
│   ├── training.py
│   ├── io.py
│   └── vision.py
└── core/                   # stable package candidate
    └── ml_core/
        ├── training/
        ├── data/
        ├── nn/
        └── utils/
```

## Intention
- Support **multiple learning topics** (topic A, topic B, topic C...)
- Keep each topic isolated and self-contained
- Reduce unnecessary duplication without creating tight coupling
- Allow reusable code to mature naturally for future projects
- Minimize effort to add new ideas

--- 

## Core Tension
When managing many topics, there is a nature tensions:
- **Topics want isolation
    - Independent notes, demos, experiments
    - Fast iteration without cross-topic coupling
- **Module want to reuse
    - Shared utlitlies and helpers shouldn't be duplicated
    - Reuse lowers maintenance and cognitive cost

Handled poorly:
- Too much isolation -> duplication
- Too much reuse -> overengineering and coupling

--

## Design Reolution: Two-Level Reuse
We resolve this by explicitly separating two kind of reuse.

---

### Level 1 Reuse: Across Topic (Fast, Unstable)

**Goal**:
Avoid copy-paste inside this repo without slowing learning.

**Where it lives**:
```
shared/
```

**Characteristics**:
- Resue across **2+ topics**
- API in **unstable by design**
- Refactoring is expected
- No heavy abstraction or packaging

**What belongs here**:
- training loops(minimal, generic)
- logging, seeding, timing
- checkpoint IO
- simple metrics / meters
- tiny dataset or transform helpers

**What does NOT belong here**:
- topic-specific model architecture
- loss functions tied to one method
- datasets used by a single topic
- "one trainer to rule them all"

**Guiding rule**
> Put **mechanics** in **shared/**, keep ideas in **topics**

### Level 2 Reuse: Stable Code for Future Projects

**Goal**:
Create code you can confidently reuse in other repos:

**Where it lives**:
```
cores
```
**Characteristics**
- Stable, boring APIs
- Clear responsibilities
- Fewer breaking changes
- Optional tests and documentations

**Promotion rule**
> Promote **shared/**-> **core/** only when reuse is proven and API stop changing.

---

### Promotion Path
Code flows naturally through three stages:
```
topics/* -> shared/ -> core/
```
- Start with copy-paste in topics
- Move small reusable pieces to **shared/**
- Promote to **core/** only when stability is earned
No step is mandatory.

---

##  Minimal Setup (Default)
To keep development lightweight:
- ❌ No modules/ by default
- ❌ No core/ until you feel real pain
- ✅ Each topic starts with just one folder: minimal/
Example:
```
topics/
└── transfer_learning/
    └── minimal/
        └── freeze_backbone.py
```
Everything else is **opt-in**.

---

## How we Avoid Cross-Topic Coupling
### Hard Rules
- **topics/** may import **shared/**
- **shared/** must **never import from **topics/**
- **shared/** export **small, composable functions**
- Avoid flag-heavy “mega abstractions”

### Practical Heuristic
- Duplicate once → ignore
- Duplicate twice → extract smallest reusable piece to shared/
- Stable + reused everywhere → consider core/

### Rules of Thumb
- Optimize for learning speed
- Tolerate duplication early
- Delay abstraction aggressively
- Refactor only when friction appears

### One-Line Summary

Topics are for learning, shared/ is for reuse without commitment, core/ is for stability — only when earned.

<!-- ## Core Principles
1. **Optimize for learning speed**
    - Easy to add new ideas
    - Refactor later
2. **Delay abstraction**
    - No "perfect" architecture up front 
    - Modularization happeds after reuse is proven
3. **Topics are the primary unit**
    - Each topic is a self-contained learning workspace

--- -->