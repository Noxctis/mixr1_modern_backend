# Chapter 5 revision

- `chapter5.tex` – your chapter with two parts replaced (the original in Downloads is untouched):
  - `subsec:agit-cetdct` (CET vs DCT) – rebuilt from `logs/ch5_new/estimators/results` (256 PPR, DCT 5/10/20 ms, CET aligned/newest).
  - `subsec:agit-plausibility` – CFD vs MIXR-1 (96 mm, 174 mm), CFD vs Armfield student data, everything vs the literature curve, with RMSE.
  - `parts/` holds the two replacement blocks on their own.
- `figures/` – PDFs/PNGs, `.tex` tables (`\input` by the chapter) and the CSVs behind every number.
  Put this folder next to `chapter5.tex` (or add it to `\graphicspath`).
- `analysis/npo_rmse.py`, `analysis/estimator_figs.py` – regenerate everything.
  `npo_rmse.py --data "<Research SeBoCa Test folder>" --out figures`; the run list is the `RUNS` dict at the top.

Open items are marked `% TODO` in the chapter (BibTeX keys for Brown and the Armfield report, baffles/liquid level of each run, reference geometry).
The chapter was not compiled (no LaTeX on this machine).
