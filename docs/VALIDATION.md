# Physical validation plan and log

Status: **no specimen has been measured.** Every results cell below is `PENDING`. An empty cell is not a zero, and it is not a pass.

This document is the check on the sag model in `sim/stability.py`. It is not a check on the choreographer, and it is not a measurement of ARMADAS hardware. The closed forms the predictions use are the module docstring and `shear_cantilever_tip_sag` / `path_chain_deflection_at`. v0.2.0 stays untagged until a round in the log has a diagnosis Joshua has written and a stiffness the code is actually fit to (`CHANGELOG.md`).

## What would change the model

The model puts all compliance in the face joints. A one-piece print is fused, so the struts themselves can bend. That is the comparison, not a printing mistake.

- Self-weight tip ratio near **3.5** supports the shear chain.
- Self-weight tip ratio near **16** supports an Euler–Bernoulli beam (uniform load, deflection ∝ `n⁴`) and says `k_shear` is the wrong compliance for this part.
- Anything else is a diagnosis, not a silent average. Joshua writes it in the round log. A code change follows that note. It is not filled in here in advance.

## Specimens

Two one-piece PLA cantilevers, one of each length. `n` is the number of **free** bays. The clamp block is the fastened root and is not counted in `n`, matching `shear_cantilever_tip_sag` (the root voxel is extra).

| | n = 3 | n = 6 |
| --- | --- | --- |
| Free bays | 3 | 6 |
| Frame pitch | 20 mm center to center | 20 mm center to center |
| Struts | 3 mm | 3 mm |
| Root | clamp block, held in a vise | same |
| Tip | tab for the hanger | same |
| Printer | Bambu P2S | Bambu P2S |
| Layer height | 0.2 mm | 0.2 mm |
| Walls | 3 | 3 |

Infill, filament, nozzle, and print orientation are not fixed in this plan. A round writes them down before any comparison. Two prints with different infill are not the same specimen.

The clamp is a finite base under the root only. Do not sit the arm on a table along its length. On the stock simulator floor the whole first course is planted, and the one-line formula does not apply (`sim/stability.py`, "Closed forms").

These are single prints, not a sample size. Replicates, if they are made, are later rounds, each with its own log entry.

## Loads and how to read them

Slotted hanger set on the tip tab. Loads from 0 g to 100 g in steps of 10 g: 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100.

At each load, photograph the tip against a ruler in the same frame as the clamp. Sag is the downward distance of a mark on the tip tab from a horizontal datum taken off the clamp block. The 0 g photo is the hanger **off**. That row is self-weight. If the tab has mass, or the hanger stays on the tab at "0 g", write that mass in the round and treat it as part of `P`. Do not assume it is zero after the fact.

Weigh the free span. `w` per bay is `(free-span mass) / n * g`. Do not use the simulator default `mass = 1`.

Photo-against-ruler error (parallax, where on the tab the mark sits, ruler tilt) belongs in the round notes. This plan does not invent a correction.

## Predictions

Weight per free bay `w = m g`. Joint stiffness in the vertical direction is `k_shear`. Hanger force `P = m_hanger g` (0 when the hanger is off).

The shear chain, from the module docstring: each spring carries the weight of every bay outboard of it. Number free bays `i = 1` at the root through `i = n` at the tip.

```
u(i) = (w / k_shear) * i * (2n - i + 1) / 2
u(n) = (w / k_shear) * n * (n + 1) / 2
```

`shear_cantilever_tip_sag(n, mass=m, g=g, k_shear=k)` is that tip value at `P = 0`.

A tip force `P` is outboard of every spring, so each of the `n` springs lengthens by an extra `P / k_shear`. That term is the same spring argument. It is not a separate function in the repository.

```
u_tip(n, P) = (w / k_shear) * n * (n + 1) / 2  +  n * P / k_shear
```

### Self-weight ratio (the 0 g row)

`w` and `k_shear` cancel:

```
u(6) / u(3) = [6 * 7 / 2] / [3 * 4 / 2] = 21 / 6 = 3.5
```

Checked against the code: `shear_cantilever_tip_sag(6) / shear_cantilever_tip_sag(3) = 3.5` for any shared positive mass, `g`, and `k_shear`.

Euler–Bernoulli cantilever, prismatic, uniform self-weight `q` (force per length): tip deflection `q L⁴ / (8 E I)`. Pitch is the same on both prints, so `L ∝ n` and `q` is the same section. The ratio is `(6/3)⁴ = 16`.

**3.5 against 16 is the unloaded comparison.** It does not need a fitted `k_shear`.

### Loaded rows

The same hanger mass on both lengths does **not** keep the ratio at 3.5. If `P` dominates self-weight, the shear-chain ratio goes to `n₆/n₃ = 2`, because the added tip sag is `n P / k_shear`. The Euler–Bernoulli point load `P L³ / (3 E I)` goes as `n³`, ratio 8. Report loaded ratios against those limits, not against 3.5.

### Fitting `k_shear`

Fit on one length. Predict the other. That is the instruction in the `sim/stability.py` docstring.

From a measured tip sag `u` at known `w` and `P`:

```
k_shear = (w * n * (n + 1) / 2  +  n * P) / u
```

Self-weight only (`P = 0`): `k_shear = w * n * (n + 1) / (2 u)`.

Preferred first fit: the n = 3 arm at one recorded hanger mass (write which row). Predict every n = 6 row, and the other n = 3 loads, with that single `k_shear`. The 0 g ratio is the check that does not use the fit. Absolute millimeters do.

A tower (`k_axial`) and a bridge (no closed form; `estimate_sag` after both stiffnesses are fit) are later rounds. They are not in the table below.

## What these prints measure, and what they do not

The n = 3 and n = 6 cantilevers, once a round is filled, give `k_shear`. That is a stiffness. It is the number in the table above. It is not a joint strength.

Face strength is a later print: a connector, loaded until the joint breaks, in compression, tension, and shear, with the break load written in the round. Moment, if the fixture can apply one, is the same kind of row. Until that print exists, every strength cell stays `PENDING`. Do not copy a stiffness into a capacity, and do not copy the ARMADAS 3×3×3 derivation (`armadas_face_compression` in `sim/stability.py`) into this table. That derivation is newtons from a published cellular strength. These PLA arms are not that specimen, and a `k = 1` solve is not in newtons.

| specimen | component | break load | unit | status |
| --- | --- | --- | --- | --- |
| connector, future print | axial compression | | | PENDING |
| connector, future print | axial tension | | | PENDING |
| connector, future print | shear | | | PENDING |
| connector, future print | moment | | | PENDING |

## Results

Blank cells are not measured. Do not fill them from the formula.

| specimen | load_g | measured_tip_mm | predicted_tip_mm | k_shear (unit in the round) | status |
| --- | --- | --- | --- | --- | --- |
| n=3 | 0 | | | | PENDING |
| n=3 | 10 | | | | PENDING |
| n=3 | 20 | | | | PENDING |
| n=3 | 30 | | | | PENDING |
| n=3 | 40 | | | | PENDING |
| n=3 | 50 | | | | PENDING |
| n=3 | 60 | | | | PENDING |
| n=3 | 70 | | | | PENDING |
| n=3 | 80 | | | | PENDING |
| n=3 | 90 | | | | PENDING |
| n=3 | 100 | | | | PENDING |
| n=6 | 0 | | | | PENDING |
| n=6 | 10 | | | | PENDING |
| n=6 | 20 | | | | PENDING |
| n=6 | 30 | | | | PENDING |
| n=6 | 40 | | | | PENDING |
| n=6 | 50 | | | | PENDING |
| n=6 | 60 | | | | PENDING |
| n=6 | 70 | | | | PENDING |
| n=6 | 80 | | | | PENDING |
| n=6 | 90 | | | | PENDING |
| n=6 | 100 | | | | PENDING |

| comparison | measured n6/n3 | shear-chain | Euler–Bernoulli | status |
| --- | --- | --- | --- | --- |
| self-weight, hanger off | | 3.5 | 16 | PENDING |
| 100 g on both tips | | 2 if P ≫ self-weight, otherwise the loaded formula above | 8 if P ≫ self-weight | PENDING |

## Round log

Copy this block for each round. **Diagnosis is written by Joshua.** Leave it blank when the round is opened. Do not draft it from the model.

### Round

- Date:
- Operator:
- Specimen: n = , pitch 20 mm, strut 3 mm, clamp block, tip tab
- Printer: Bambu P2S, layer 0.2 mm, walls 3
- Infill:
- Filament:
- Nozzle:
- Orientation on the bed:
- Free-span mass (g):
- Tab or hanger mass included at 0 g (g):
- How the ruler photo was read (datum, mark on the tab):
- Measured: which rows of the table this round filled
- Predicted: `k_shear` fit on (specimen, load) = ; formula `u_tip` above; code reference `shear_cantilever_tip_sag` for `P = 0`
- Diagnosis (by Joshua):
- Code change:
