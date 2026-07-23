# commit reword map (2026-07-23)

The 59 unpushed commits on feat/bernstein-gate got human-readable subjects on 07-23
(bodies untouched, trees byte-identical pair by pair -- verified). The old chain is
pinned forever at tag `pre-reword-2026-07-23`; sealed artifacts (final100 scenarios/
receipts, calib_v6_final100, plans, baseline/mem_k ledgers) still cite OLD shas and
were deliberately NOT edited. Freeze tags estimator-freeze-2026-07-20 and
stack-freeze-m3off-2026-07-21 still point into the old chain -- that is correct.

| old | new | subject (new) |
|-----|-----|---------------|
| 213a19d | 13e9269 | fix a batch of cert-entry audit findings: crashes, silent eps mismatch, overflow guards |
| d32b63b | 9d974de | measure clearance from the body, not the centre -- and against movers at the same instant |
| a93ab2a | 8388066 | fly exactly the gear we certified, no extra smoothing on top |
| d4e66eb | 87ed0f4 | rebuild cert_capi.so to pick up 13e9269 |
| 5b08111 | 3984726 | notes: write up the audit verdict and the fix campaign |
| d87aa83 | 5343faa | evening decisions: keep the verbatim executor, k10 goes default, new 6-seed baseline |
| 226698a | 017d4a2 | honesty check on the v6 calibration -- turns out the 5% tube was really a 10% tube |
| 2f978ff | 029836a | decision receipts + clearance swept over the whole tick, not just endpoints |
| 73ab731 | 84ae78d | goldens for 029836a |
| 74e72c4 | 188028a | redo the conformal ladder honestly, interim eps=0.10 artifact |
| ee4e94e | 4777780 | notes: book the late-night ruling |
| b8e28b2 | 268a3f8 | collision attribution (U/P/Q/D), one shared module wired into both faces |
| 7eed9d7 | 6df07d0 | goldens for 268a3f8 |
| cfbf6a9 | 4563d95 | notes: attribution steps 1-4 done |
| 2dc2176 | e93df80 | five little holes in the proof chain, all closed, no behavior change |
| 1c32963 | f0a78e0 | notes: proof-chain hardening booked |
| 3f64cf6 | d2fb9c0 | two KF fixes: coast no longer drifts on stale accel, tracks stop dying at the cone edge |
| 1667250 | e78c564 | goldens for d2fb9c0 |
| fe79734 | ab9127a | notes: the estimator knives, booked |
| 25ee7e4 | d54675f | make the estimator internally consistent -- coast mean+cov under one CV model, TTLs in seconds everywhere |
| 1a80c30 | ca11668 | goldens for d54675f |
| 1596549 | 68d5a5b | notes: M3 verdict is FAIL, estimator frozen at d54675f |
| c3a4702 | a84e0f5 | make harvesting trustworthy: atomic episodes, pre-registered jobs, fail-loud everywhere |
| 793c4ed | 2c10526 | found and killed the harvest episode-id off-by-one (it was there since the field's birth) |
| 514afb7 | cfd3575 | notes: harvest contract validated end to end |
| ba5775b | e7239dd | pick EGO_MEM_K from the design69 data: keep 2.0, vehicles get 13.5 |
| 34966e7 | d9ed304 | notes: design69 re-harvest and the K table |
| d551476 | b9b9078 | walk back vehicle K=13.5 -- the fit was answering the wrong question |
| df49ac0 | 4dd3ddf | the cert core can never silently shrink a time window again |
| ccb7cc0 | fea940b | zero-row episodes now vote in the calibrator + final100 gets pre-registered |
| c811e46 | 6196fec | notes: triple-veto ruling executed |
| 50f9f85 | f783ea3 | close the last cert seams: one shared stop point, hover tails checked everywhere |
| 073e187 | 7cab276 | freeze the stack: sha manifest for every .so, final100 refuses anything off-plan |
| 7fa0240 | bbfdfd6 | notes: second triple-veto booked |
| e3b8432 | 0eb1dca | close the metaurban env before opening the next one (engine singleton bit us live) |
| 9388e15 | 9b045a1 | widen the clear-start search -- half the campaign was dying at takeoff |
| 07207c0 | 3f166cf | un-break the contested families: retry each corridor, retime markers exactly |
| 1f74270 | 2c89474 | the paper calibrator: cal60 rank, test split opens exactly once |
| 2e7703e | 85f7cb9 | final100 is in: 100/100 generated, calibrated, test opened once and sealed |
| 0eaf46b | c0edfe8 | notes: launch-day wrap-up |
| 4f46ca6 | 81577ab | draft of the paper methods section |
| 95bffcb | 6d193e3 | every refusal now says who killed it and why (EGO_EXPLAIN) -- and it promptly overturned the GT-xy story |
| 41816cd | 1acf34a | notes: the GT-xy case, full write-up |
| 914bca0 | e5217a6 | notes: the four-link death chain, pinned with file:line receipts |
| 6d9c6a3 | 6030281 | notes: full-stack re-audit ledger (sweep21 vol.1) |
| dea9f0b | 751b787 | notes: sweep21 archive -- KF 5/20 vs GT 15/20, and all the GT dirt is liveness |
| d0a55aa | 0154975 | notes: north-star erratum (GT_XY is not an oracle) |
| 6ab60f8 | 15fe723 | the guide arm: KF prediction draws a line, EGO just flies it |
| 04a37c4 | 9f87937 | guide fixes after acid test 1: radii finally carry tube growth, plus a per-stage failure trail |
| 9324f31 | a7ad1c5 | let the guide switch sides when its own side is capped out -- and kick off the 0-hold push |
| 5f60786 | dd645bd | notes: guide arm campaign archive |
| 0b671eb | 92ddb16 | big guide rework so gtxy stops holding -- also A* is finally deterministic |
| db6ef38 | 5614b42 | slow through the pinches instead of stopping (certified gear ladder) |
| 50ebffc | b258196 | jitter forensics + make roll2 sim from where we actually are (s14 finally clean) |
| 34ab1a8 | 8d5813c | built a proper 2nd-order lateral tracker, measured it, shipping it off by default |
| 9377c6b | 72c36f9 | found the jitter root: the smoothness term literally cannot see accel at ts=0.075 (EGO_CPD) |
| 3668c16 | ed8d195 | tried pricing physical accel in the objective -- it just moves the noise around (kept, off) |
| 6b2b37c | 331728c | zero-phase denoiser for accepted plans -- works, but the real enemy is chaos, not noise |
| 1b033bd | f55526a | cleanup: drop dead code, dedup the hot paths. checked byte-identical |
