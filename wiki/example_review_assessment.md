# Example design reviews: summaries and rubric assessment

Assessed 30 September 2026. Sources: the nine PDFs in the project's Example Reviews Drive folder and the supplied PDR and CDR mark schemes.

## Main findings

SpaceEX is the clearest starting point for structuring a UK PDR/CDR. Sand Hunters is the most directly useful reference for this project's camera and image-processing mission. Herotech is the best reference for explaining engineering revisions. AstroSat provides the most useful quantitative parachute comparison, with important weaknesses in how it interprets the measurements. Luke's CDR provides a valuable caution: a substantial amount of practical work can still receive a low report mark.

## Assessment method and limits

- PDRs use the supplied **36-point PDR scheme**; CDRs use the supplied **34-point CDR scheme**.
- All scores are my indicative assessments except **Luke CDR: 14/34**, which is explicitly recorded in the supplied CDR scheme. Its selected criterion marks have been transcribed, not reinterpreted.
- For estimated scores, treat a difference of roughly 2-3 marks as uncertain. These are evidence-based point allocations, not official judge grades or statistical confidence intervals.
- No FDR scheme was supplied. LCSat and AstroSat are therefore rated against the CDR scheme as a provisional comparison, not given an official FDR score.
- Reports span different countries and years. I apply the supplied UK criteria to the documents as provided; this does not establish how they should have scored in their original competitions.
- Format limits are 20 main-report pages for PDR and 25 for CDR, with appendices excluded. Brighton's supplied PDF has 30 pages and contains an embedded later CDR ground-support section. Sand Hunters has 29 pages, Herotech 31, and AstroSat 40, without sufficient material explicitly designated as appendices to fit the relevant limits. These receive 0 for the reporting-template criterion as supplied. A shorter, reorganised submission could recover those marks.
- I read the complete extracted text and reviewed all pages visually, including diagrams, prototype photographs and highlighted rubric selections. External links within reports were not followed; an external resource is not credited as evidence unless its relevant content is visible in the supplied PDF.
- The PDR scheme also contains highlighted example marks totalling **23/36**, but does not identify the team. I do not attribute those marks to Luke or another report.

Rubrics: [PDR mark scheme](https://drive.google.com/file/d/11Vy_lmZl5IOop-UIJo3-M98J3fZ7pV9n/view), [CDR mark scheme](https://drive.google.com/file/d/1UFGm9K5-NEfBNUXjkNIA73CDiApQ270h/view).

## Score overview

| Review | Assessment | Percentage | Most useful feature |
|---|---:|---:|---|
| SpaceEX PDR | 31/36, estimated | 86% | Clear UK structure, schedule and software diagrams |
| Sand Hunters PDR | 29/36, estimated | 81% | Imaging pipeline, state machine, antenna work |
| Brighton PDR | 27/36, estimated | 75% | Scientific hypothesis and mechanical/material trade-offs |
| Luke PDR | 23/36, estimated | 64% | Parachute mechanics and engaging outreach plan |
| SpaceEX CDR | 27/34, estimated | 79% | Traceable structure, diagrams and prototype evidence |
| Herotech CDR | 23/34, estimated | 68% | Detailed reasons for hardware revisions |
| Luke CDR | **14/34, recorded grade** | **41%** | Iterative build/test record; judge-marking caution |
| AstroSat FDR | 24/34, provisional CDR comparison | 71% | Quantitative parachute testing and data workflow |
| LCSat FDR | 22/34, provisional CDR comparison | 65% | Lessons learnt, system integration and software diagrams |

## PDR criterion breakdown

| Criterion | Max | SpaceEX PDR | Sand Hunters PDR | Brighton PDR | Luke PDR |
|---|---:|---:|---:|---:|---:|
| Reporting template | 2 | 2 | 0 | 0 | 1 |
| Team organisation and roles | 2 | 2 | 2 | 1 | 1 |
| Primary mission | 3 | 3 | 3 | 3 | 3 |
| Secondary mission | 3 | 2 | 3 | 3 | 2 |
| Project plan | 3 | 3 | 3 | 3 | 1 |
| Team competencies/support | 3 | 2 | 1 | 3 | 2 |
| Risk analysis | 3 | 3 | 3 | 3 | 2 |
| Mechanical description/diagram | 1 | 1 | 1 | 1 | 1 |
| Mechanical materials | 2 | 2 | 2 | 2 | 2 |
| Electrical design | 3 | 2 | 3 | 2 | 1 |
| Software design | 3 | 3 | 3 | 1 | 1 |
| Ground support equipment | 2 | 2 | 2 | 2 | 1 |
| Testing plan | 3 | 1 | 1 | 2 | 2 |
| Outreach plan | 3 | 3 | 2 | 1 | 3 |
| **Total** | **36** | **31** | **29** | **27** | **23** |

Scores reflect rubric coverage, not a ranking of engineering ambition. In particular, format penalties and missing diagrams are independent of how interesting a mission is.

### SpaceEX PDR

[Source PDF](https://drive.google.com/file/d/1pWWHiXsFEhzXDOkDFT3HPWrL58iKFeTp/view). Lancing College, dated 20 November 2024; 15 pages.

**Mission:** Pressure/temperature telemetry and GPS tracking to reconstruct descent and infer wind or turbulence, with Blender used to visualise trajectory and sensor values.

**Strengths:** Responsibilities and relevant experience are explained on p.3. A readable Gantt chart and competence matrix appear on p.5. CAD and structural arrangement are shown on pp.6-8. CanSat, receiver and data-presentation flowcharts on pp.10-11 make the software approach easy to follow. Outreach on p.15 justifies both methods and audiences.

**Mark losses:** The secondary mission needs a clearer question, success threshold and account of how GPS uncertainty affects the inferred wind. Competence gaps and the route to obtaining support are less explicit than the team-skills table. Electronics identify parts and justify the familiar Arduino platform, but sensor limitations are only lightly addressed. The testing list on p.14 is useful but is not a complete requirements-to-test matrix with numerical acceptance criteria.

**Use for our project:** Report structure and separate diagrams for onboard acquisition, receiver operation and post-processing.

### Sand Hunters PDR

[Source PDF](https://drive.google.com/file/d/1lt5vIELyCHpQ6fl_OQ_k_D7StZXn5_M4/view). Romanian competition, dated 26 February 2023; 29 pages.

**Mission:** Atmospheric measurements plus downward images, processed after recovery using machine learning to identify agricultural areas, with possible extension to desertification and other land classes.

**Strengths:** Roles are mapped visually on p.3. Vertical and horizontal mechanical layouts are compared on pp.5-7. Electrical blocks, sensor specifications and interfaces are documented on pp.8-10. Antenna modelling on pp.11-14 is unusually developed. State machines and the image-processing pipeline on pp.14-17 cover both missions. Tasks are scheduled and assigned on pp.21-22. Software tests on pp.23-25 include an 80/20 dataset split and accuracy/F1 evaluation.

**Mark losses:** It exceeds the supplied UK PDR page limit. The team does not clearly identify gaps in its own competencies and arrange technical support for them. The extensive test plan is not explicitly mapped to all key UK CanSat requirements; attachment-load and launch-load verification need clearer coverage. Some acceptance tests use subjective terms such as satisfactory quality or accuracy. Outreach is justified but not especially developed.

**Use for our project:** Closest imaging precedent, especially image capture, preprocessing, evaluation and state-machine structure. Add numerical success thresholds and a shorter main report.

### Brighton PDR

[Source PDF](https://drive.google.com/file/d/1V1lkajUyh5RIXY1O0ickR8siO4mZaffZ/view). Celestium, Brighton College, dated 22 November 2024; supplied PDF has 30 pages.

**Mission:** Measure UV/IR and ozone/CO2 against altitude to investigate correlations associated with atmospheric filtering; deployable spoilers supplement parachute descent control.

**Strengths:** An explicit scientific hypothesis is stated on PDF p.4. A visible schedule and competence matrix are on p.5. Risk mitigations span manufacturing, electronics, software and environmental conditions on p.6. PLA/TPU choices and shock absorption are justified on pp.6-7. Circuit and interface discussion is substantial on pp.8-10, and parachute material comparisons on pp.15-16 are useful. Ground-support choices are extensively explained.

**Mark losses:** The supplied file mixes PDR material with a later section headed “CDR 3.5 Ground Support Equipment” on pp.19-27 and exceeds the PDR limit without appendices. Roles are named without much explanation of responsibilities. Software is discussed through code and prose, but lacks the required basic block diagrams for both missions. Outreach on pp.29-30 is mostly a list of possibilities and an Instagram commitment rather than a clear audience-based plan.

**Scientific limitation:** The report proposes an interesting correlation, but does not show whether the selected sensors, altitude range and sampling conditions can resolve the expected effect. This is a feasibility question to address before treating the proposed outputs as scientifically convincing.

**Use for our project:** Mechanical/material trade-offs and a clearly stated hypothesis. Remove the mixed-stage material and replace long code explanations with diagrams.

### Luke PDR

[Source PDF](https://drive.google.com/file/d/1LqMyPr2i8_e4rF9ZhyAgCIotEO9HEBq4/view). DHSB Gravity Hackers, dated 20 November 2025; 14 pages.

**Mission:** Temperature/pressure measurements, plus magnetic field and acceleration against altitude, with Physics Society involvement in post-flight analysis.

**Strengths:** A mechanical sketch, frame/material explanation and parachute calculations appear on pp.5-7. Competence gaps, including lack of radio expertise, are acknowledged on p.4. Drop, mount-load and software tests are discussed on pp.8-9. Outreach on pp.13-14 combines school communications, social media, Physics Society and a Year 7 exterior-design competition.

**Mark losses:** Role descriptions are brief. The work-hour list on p.3 does not provide a dated schedule or dependencies. Risk coverage and mitigations are limited. Electronics lack a developed explanation of processor choice, sensor limitations and compatibility. Software has no block diagram. Ground-support choices are mostly listed rather than justified. The test plan is broad but needs a complete requirements checklist and clearer pass/fail thresholds. Photographs of students would need checking against the supplied CDR instruction excluding personal information if carried forward.

**Use for our project:** Practical mechanics and participatory outreach; strengthen scheduling and software presentation.

## CDR criterion breakdown, including provisional FDR comparisons

| Criterion | Max | SpaceEX CDR | Herotech CDR | AstroSat FDR (CDR proxy) | LCSat FDR (CDR proxy) | Luke CDR (recorded grade) |
|---|---:|---:|---:|---:|---:|---:|
| Reporting template | 2 | 2 | 0 | 0 | 1 | 2 |
| Team organisation and roles | 2 | 2 | 2 | 2 | 2 | 1 |
| Primary mission | 3 | 3 | 2 | 3 | 1 | 1 |
| Secondary mission | 3 | 2 | 3 | 2 | 2 | 1 |
| Project plan | 3 | 3 | 3 | 2 | 2 | 1 |
| Team competencies/support | 3 | 2 | 1 | 1 | 3 | 2 |
| Risk analysis | 3 | 3 | 1 | 1 | 2 | 1 |
| Mechanical design | 2 | 2 | 2 | 2 | 2 | 1 |
| Electrical design | 3 | 2 | 3 | 3 | 1 | 1 |
| Software design | 2 | 2 | 1 | 1 | 2 | 0 |
| Ground support equipment | 2 | 2 | 2 | 2 | 1 | 1 |
| Testing results | 3 | 1 | 2 | 2 | 2 | 1 |
| Outreach | 3 | 1 | 1 | 3 | 1 | 1 |
| **Total** | **34** | **27** | **23** | **24** | **22** | **14** |

The Luke column is the actual selected marks in the supplied scheme. The other columns are my assessments. They are not evidence that a judge would award the same marks.

### SpaceEX CDR

[Source PDF](https://drive.google.com/file/d/123aFPZMcUQtkvODHEq83ITKSP2K6H3rB/view). Lancing College, dated 30 January 2025; 23 pages.

**Mission:** Develops the PDR's GPS descent reconstruction, using temperature, pressure and position telemetry and a Blender visualisation.

**Strengths:** The Gantt chart on p.6 shows planned and completed work. Safety and mission risks are separated on pp.7-9. Wiring/PCB evidence appears on pp.11-12. Flowcharts for the CanSat, receiver and Blender processing are on pp.13-15. Prototype photographs and a results table on pp.19-22 make progress visible.

**Mark losses:** The test results on p.19 are largely qualitative: survived, worked, LEDs blinked or values matched. A power-on test is not a battery-endurance test, and a working bench radio link does not establish flight-range performance. The full drop test is still planned. The report names different radio modules in different sections, so the final configuration needs reconciliation. Secondary-mission success criteria remain vague. Competence-gap support and measured outreach reach/impact are weak.

**Use for our project:** Best CDR organisation reference, while replacing informal test claims with numerical, requirement-linked results.

### Herotech CDR

[Source PDF](https://drive.google.com/file/d/1HOfgeM7dTy8gjzS0_DyZUem5VTgV5V82/view). Romanian competition, dated April 2021; 31 pages.

**Mission:** Guided landing within a 25 m target radius using a steerable paraglider, GPS/IMU feedback and four motor-driven lines.

**Strengths:** A schedule appears on p.3 and team responsibilities on pp.4-5. Mechanical and electronic revisions are explained in considerable detail. The change from PLA to ABS follows measured heating on p.9. PCB sections on pp.13-24 describe USB backfeeding, soldering difficulties, GPS/antenna failures and design responses. Measured current and battery lifetime on pp.24-25 support the discussion: approximately 2.1 hours under the reported active load and 10 hours in power-saving mode.

**Mark losses:** The report exceeds the UK CDR page limit and includes personal information. Competence gaps/support arrangements and systematic risk analysis are weak. The software diagram on p.25 is high level rather than a detailed diagram of all major subprograms. Primary telemetry requirements are not prominent; the original competition treated radio as optional. The final paraglider and guided-landing demonstration are unresolved, and large-scale tests are plans rather than achieved results. Outreach describes coverage but does not quantify reach or impact.

**Use for our project:** Strongest example of explaining an engineering change as problem, evidence, alternatives and decision.

### Luke CDR: recorded 14/34

[Source PDF](https://drive.google.com/file/d/1tO71oHFY8RkprfC71tSIWJeiI9SRoJfC/view). DHSB Gravity Hackers, dated 30 January 2026; 21 pages.

**Mission:** Magnetic-field and acceleration profiling alongside the primary atmospheric mission.

**Strengths:** Dated tests on pp.14-17 document genuine iteration: stronger columns/infill, print orientation, mount changes after knot failure, and radio tests to 335 m and then 600 m. Complete prototype photographs appear on p.19. The report is useful evidence of sustained practical work.

**Recorded marking:** The supplied CDR scheme identifies DHSB Gravity Hackers and reports **14/34, graded 27 February 2026**. Highlighted selections show 0/2 for software and 1-point awards for primary mission, secondary mission, project plan, risks, mechanical design, electrical design, ground support, testing and outreach. Template receives 2/2, roles 1/2 and competencies 2/3.

**Interpretation:** The report's practical effort did not translate into high criterion marks. It lacks a proper software flowchart and dated project schedule, and the primary-mission overview does not explicitly state the telemetry requirement. Scientific justification, test-to-requirement traceability and outreach metrics could be clearer. These are my explanations of likely rubric gaps; the visible general feedback is truncated, so they are not claimed as verbatim judge reasons.

**Use for our project:** Copy the dated failure/change/retest record, then make each scoring criterion explicit in the report.

### AstroSat FDR: provisional CDR comparison

[Source PDF](https://drive.google.com/file/d/1dAe3f8OvbBqrhuWUE6c6zwSipTplHbFg/view). Warsaw, Poland, dated 25 March 2025; 40 pages.

**Mission:** Assess environmental conditions using pressure, temperature, humidity, UV, oxygen, CO2, acceleration and images, framed as a planetary-habitability investigation.

**Strengths:** A change log on p.3, exploded CAD/assembly photographs on pp.8-13, and mass/power budgets make the implementation visible. Radio fallback architecture is described on pp.21-22. Software fault handling and post-flight merging of onboard/ground data are developed. Parachute tests on pp.29-33 compare canopy sizes and masses, including revision of the assumed drag coefficient. Outreach on pp.37-39 uses a coherent WALL-E story and several different publications/audiences.

**Mark losses:** The 40-page main report exceeds the CDR limit. Roles are explained, but skill-gap support and risk analysis are not developed. Software is explained mostly in prose/data-flow architecture, rather than detailed block diagrams. The radio range test was not completed (p.33), and camera integration had failed (p.19). Descriptions of sensors, frequencies and acquisition intervals are inconsistent between sections.

**Testing limitation:** Height divided by total fall time is an average descent speed, not a direct measurement of terminal velocity. The report calls this terminal speed. It also lists incompatible summaries: 14.801 m divided by 4.8 s is about 3.08 m/s, rather than the 3.78 m/s reported for the 56 cm test. For the 45 cm canopy it lists different aggregate speeds while the near-flight-mass test gives 6.43 m/s. These need resolving before using the figures as validation.

**Use for our project:** Quantitative test tables and theory/experiment comparison, with explicit measurement uncertainty, consistent configuration and a validated camera system.

### LCSat FDR: provisional CDR comparison

[Source PDF](https://drive.google.com/file/d/1IhUArhBlwQP5bUH87BxoSLonQDanrV7S/view). France, 2021-era final report; 20 PDF pages, with appendices starting on PDF p.18.

**Mission:** Record several environmental/motion signals and two camera streams, then explore correlations using a web application and optional encryption.

**Strengths:** Responsibilities and external help are explained. The software is organised into onboard and ground applications, with detailed flowcharts in Appendix 5 (p.20). Testing on pp.13-14 describes unit and integration phases with failures found and corrected. Lessons learnt on p.15 describe purchasing delays, replacement of the onboard computer and contingency SD cards.

**Mark losses:** It contains personal details and uses an older non-UK template. The primary mission emphasises onboard recording and transfer after recording rather than demonstrating live telemetry during descent. The secondary mission is broad correlation-finding without a focused hypothesis. Electronics have a high-level architecture but lack complete circuit diagrams. Risks are addressed retrospectively rather than through a full risk register. Tests report successful outcomes but little numerical verification. Ground-radio range and outreach reach/impact are not established.

**Use for our project:** Lessons learnt, unit-to-integration testing and software flowcharts. Its data-transfer approach should not be adopted as proof of the UK live-telemetry requirement.

## What to take into our own review

1. Build the main report around the actual rubric, using headings that make every criterion easy to find.
2. State the primary mission explicitly: what is measured, required cadence, radio transmission and ground-station reception. Show the complete data path.
3. Give the secondary mission a measurable success criterion, not just a topic. For terrain-relative navigation, specify what position-error measure will be evaluated and how ground truth will be obtained. For image-derived wind, specify what will be estimated and the independent measurement used for comparison.
4. Include separate, legible software diagrams for flight operation, ground reception and image/data analysis.
5. For each requirement, record the test method, conditions, result, pass/fail decision and consequent design change. At CDR, plans alone cannot substitute for achieved results.
6. Show an assembled working CanSat, a circuit diagram and explicit evidence of hardware/software integration.
7. Include a dated schedule, competence gaps with named support routes, and risks with consequences and mitigations.
8. Record outreach actually delivered and its audience/reach/impact. A long list of channels is weaker evidence than measured outcomes.
9. Keep the main report within its page limit; place detailed code, datasets and extended calculations in labelled appendices with references from the main report.

These reviews and rubrics are historical reference material. This assessment does not establish the requirements or marking for the 2027 competition.

