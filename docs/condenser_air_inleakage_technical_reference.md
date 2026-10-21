---
title: Steam Surface Condenser Air In-Leakage - Thermodynamics, Diagnostics, and MML Modeling
description: Portable engineering reference for interpreting condenser air in-leakage, noncondensable partial pressure, turbine backpressure, heat-transfer degradation, and the Metroscope Modeling Library condenser fault.
tags:
  - steam-surface-condenser
  - air-inleakage
  - noncondensables
  - turbine-backpressure
  - nuclear-power
  - metroscope-modeling-library
last_verified: 2026-09-10
---

# Steam Surface Condenser Air In-Leakage

## Purpose and limits

This entry summarizes the thermodynamic reasoning, diagnostic limitations, literature correlations, and MML implementation details relevant to air in-leakage in a nuclear-plant steam surface condenser.

It is intended as:

- a technical reference for condenser-performance discussions;
- context that can be supplied directly to Codex or ChatGPT;
- a basis for improving the `air_intake` fault in the Metroscope Modeling Library (MML); and
- a guide to what can and cannot be concluded from condenser pressure and condensate-temperature tags.

It is not a substitute for the plant's condenser design data, HEI/ASME performance calculation, calibrated instrumentation, air-removal-system curves, or plant procedures.

## Executive conclusions

1. A turbine exhausts against the condenser's **mixture static pressure**:

   $$
   P_{\mathrm{cond}}=p_s+p_a,
   $$

   where $p_s$ and $p_a$ are the steam and air partial pressures. Partial pressure controls steam phase equilibrium; the sum controls the mechanical backpressure seen by the turbine.

2. Air can reduce turbine power even if $p_s$ initially remains unchanged. At fixed steam flow, higher $P_{\mathrm{cond}}$ reduces the turbine's enthalpy drop and work per kilogram. A flow reduction is not required.

3. In a real condenser, air often also increases $p_s$ and the corresponding steam saturation temperature. Air creates a diffusion barrier near condensing surfaces and reduces the steam-side condensation coefficient. The higher turbine exhaust enthalpy can also increase condenser duty. With finite area and cooling capacity, the plant commonly settles at a higher condensing temperature.

4. Dry air is essentially ideal over approximately 0-5 inHgA and 32-80 degF. The ideal-gas equation is not the important uncertainty. The unknown gas inventory, leakage rate, removal rate, spatial distribution, and temperature are.

5. A negative empirical correlation between inferred noncondensable pressure and condenser saturation temperature is inconsistent with a **constant-air-concentration** model, but it does not prove that no air is present.

6. A condensate outlet or hotwell temperature is not automatically the condenser vapor equilibrium temperature. Subcooling, drain mixing, recirculation, stratification, sensor location, and calibration can all separate the two.

7. MML correctly adds air partial pressure to the pressure presented to the turbine, but its current `air_intake` fault is an imposed concentration rather than a leak flow or dynamic inventory. It also omits the direct reduction in condensation heat transfer caused by air.

8. At 3 inHgA and a uniform air concentration of 0.01 mol/m3, the EPRI/Henderson-Marchello correlation predicts approximately an **11.7% reduction in steam-side film coefficient**. A reasonable illustrative reduction in whole-condenser $U$ is approximately **3-6%**, nominally about **4%**, depending on the steam-side share of total thermal resistance.

## 1. Pressure, composition, and equilibrium

### 1.1 Dalton pressure balance

For a low-pressure steam-air mixture,

$$
P_{\mathrm{cond}}=p_s+p_a.
$$

Here, "total" means the Dalton-sum static pressure, not turbomachinery stagnation pressure.

If the steam is locally in equilibrium with liquid water at vapor temperature $T_v$,

$$
p_s=P_{\mathrm{sat}}(T_v).
$$

Consequently,

$$
p_a=P_{\mathrm{cond}}-P_{\mathrm{sat}}(T_v).
$$

This relation is rigorous only when the pressure and temperature describe the same representative location and the steam is close to local phase equilibrium.

### 1.2 Air equation of state

For air,

$$
p_a=Zc_aRT_a,
$$

where:

- $c_a=n_a/V$ is air molar concentration;
- $T_a$ is air temperature;
- $Z$ is compressibility factor; and
- $R$ is the universal gas constant.

Equivalently,

$$
c_a=\frac{p_a}{ZRT_a},
\qquad
\rho_a=M_a c_a.
$$

Waxman and Davis measured ultra-pure air using an isothermal Burnett apparatus. At 298.15 K and 0.01 MPa, close to 3 inHgA, they reported $Z=0.99997$. Therefore the ideal-gas error is only about 0.003% at that state. The more comprehensive NIST Helmholtz-energy equation of state by Lemmon et al. was fitted to experimental pressure-density-temperature and other thermophysical data and covers a vastly larger range than condenser service.

For the present condenser regime, using $Z=1$ is normally far more accurate than the plant measurements and spatial assumptions.

### 1.3 What an equation of state does not determine

An equation of state relates local equilibrium state variables. It does not determine how the inventory in an open condenser changes with time.

For a gas volume $V_g$,

$$
\frac{dn_a}{dt}
=\dot n_{\mathrm{leak}}
-\dot n_{\mathrm{removal}}
-\dot n_{\mathrm{dissolution}},
$$

and

$$
c_a=\frac{n_a}{V_g}.
$$

Waxman and Davis used a confined gas sample and stepwise expansions between calibrated volumes because that is how accurate PVT data are obtained. This does not make their equation-of-state information invalid for an open condenser. It means only that a separate mass balance is required to determine the condenser's path through the PVT state surface.

## 2. Expected pressure-temperature relationships

From $p_a=Zc_aRT_a$,

$$
d\ln p_a=d\ln Z+d\ln c_a+d\ln T_a.
$$

At condenser pressures, $Z\approx1$, so

$$
\frac{dp_a}{dT_a}
\approx R\left(c_a+T_a\frac{dc_a}{dT_a}\right).
$$

Therefore:

- **Constant concentration:** $p_a=c_aRT_a$, giving a positive, nearly linear pressure-temperature relationship.
- **Constant inventory and volume:** concentration is constant, so the same positive relationship applies.
- **Constant pressure:** $c_a=p_a/(RT_a)$, so density decreases approximately as $1/T_a$.
- **Open condenser:** the sign is not fixed because $c_a$ depends on leakage, air removal, gas volume, condensation scavenging, load, and operating actions.

The condition for a negative pressure-temperature slope is approximately

$$
\boxed{
\frac{dc_a}{dT_a}< -\frac{c_a}{T_a}
}
$$

or, including real-gas behavior,

$$
\frac{d\ln c_a}{dT_a}
<-rac{1}{T_a}-\frac{d\ln Z}{dT_a}.
$$

Because $Z$ is essentially unity here, a consistently negative $p_a$-versus-$T$ trajectory means that air concentration is falling with temperature strongly enough to overwhelm the direct $RT$ effect. The air EOS alone cannot say whether that occurs in a particular condenser.

Practical causes of an inverse trajectory include:

- a leakage rate that increases as condenser vacuum becomes stronger at lower temperature;
- air-removal capacity changing with suction pressure, seal-water temperature, motive-steam conditions, or equipment staging;
- operator switching of ejectors or vacuum pumps;
- load-dependent steam sweeping of noncondensables toward the air-removal section;
- seasonal covariance among circulating-water temperature, plant load, condenser pressure, and removal-system performance;
- changing air-bound tube area or spatial redistribution of air; and
- temperature-proxy error caused by seasonal subcooling, hotwell mixing, or sensor drift.

## 3. Why total condenser pressure reduces turbine power

For a turbine with inlet state $(h_1,s_1)$, the ideal outlet enthalpy is evaluated at the mechanical exhaust pressure:

$$
h_{2s}=h(P_{\mathrm{cond}},s_1).
$$

Actual turbine power is approximately

$$
\dot W_t
=\dot m_s\eta_t
\left[h_1-h_{2s}(P_{\mathrm{cond}},s_1)\right].
$$

Increasing $P_{\mathrm{cond}}$ increases $h_{2s}$, reducing specific turbine work. Thus power can decrease at unchanged steam and feedwater flow.

Partial pressure does not provide a separate mechanical boundary that turbine blades can selectively feel. The wall or exhaust plane experiences the sum of momentum contributions from all gas species.

### 3.1 How steam goes from a 5-psia turbine state to 4-psia partial pressure

Suppose the condenser mixture has

$$
P_{\mathrm{cond}}=5\ \mathrm{psia}
=4\ \mathrm{psia\ steam}+1\ \mathrm{psia\ air}.
$$

A useful spatial interpretation is:

$$
\begin{array}{c}
\text{Last-stage exit: }P\approx5,\ y_s\approx1,\ p_s\approx5\\
\downarrow\ \text{irreversible mixing, diffusion, and condensation}\\
\text{Steam-air region: }P\approx5,\ y_s=0.8,\ p_s=4,\ p_a=1.
\end{array}
$$

The 5-to-4 change is not a mechanical pressure drop of the entire flow. Steam's partial pressure falls because its mole fraction falls:

$$
p_s=y_sP=0.8(5)=4\ \mathrm{psia}.
$$

The turbine exhaust enthalpy remains associated with expansion to the 5-psia mechanical boundary. Mixing with air does not provide a second reversible expander. The associated mixing and diffusion are irreversible, and the remaining energy is rejected in the condenser.

At fixed steam flow, higher backpressure also produces higher turbine exhaust enthalpy, increasing the required condenser duty:

$$
\dot Q_{\mathrm{cond}}
=\dot m_s\left[h_{\mathrm{exhaust}}-h_f(p_s)\right].
$$

## 4. Why air commonly raises steam partial pressure too

Air affects a real condenser by two coupled routes:

1. **Direct backpressure:** $p_a$ adds to $p_s$, increasing the turbine's mechanical exhaust pressure.
2. **Heat- and mass-transfer degradation:** steam condenses at the tube surface while air does not. Air becomes enriched near the interface, forming a diffusion layer through which steam must travel.

The local air layer reduces the condensation film coefficient. At fixed duty, circulating-water conditions, and available area, the condenser must operate with a larger temperature difference. Steam inventory and $p_s$ commonly rise until the necessary heat transfer is restored.

Consequently, the common plant response is

$$
p_a\uparrow
\Rightarrow P_{\mathrm{cond}}\uparrow
\Rightarrow \text{turbine specific work}\downarrow,
$$

with an additional coupled response

$$
h_{s,\mathrm{eff}}\downarrow,
\quad U_{\mathrm{eff}}\downarrow,
\quad T_{\mathrm{sat}}\uparrow,
\quad p_s\uparrow.
$$

The rise in $p_s$ is common but is not required by Dalton's law. An ideal condenser with effectively unlimited heat-transfer capacity could hold vapor temperature and $p_s$ fixed while $p_a$ and total backpressure rose.

## 5. Condensate temperature is not necessarily vapor temperature

The equilibrium calculation requires the representative vapor temperature $T_v$, not merely any condensate temperature tag.

Normally, falling condensate can be subcooled relative to bulk vapor because it remains in contact with colder tubes. Air-removal regions can be colder still. Conversely, hot drains, bypasses, makeup, recirculation, or poor sensor location can make a hotwell or outlet measurement exceed the inferred condenser saturation temperature.

Therefore, a tag such as `COND_FROM_CONDENSER_TEMP` may track the condenser's trajectory without accurately reporting the absolute vapor equilibrium temperature.

## 6. Assessment of the pressure-temperature diagnostic

The proposed diagnostic is

$$
\widehat p_a
=P_{\mathrm{cond}}
-P_{\mathrm{sat}}(T_{\mathrm{cond,out}}+T_{\mathrm{offset}}).
$$

A negative temperature offset can enforce the necessary inequality

$$
P_{\mathrm{sat}}(T_{\mathrm{proxy}})<P_{\mathrm{cond}}.
$$

This is a useful exploratory sensitivity calculation, but it does not make the inferred $\widehat p_a$ uniquely physical.

### 6.1 What a negative correlation proves

If

$$
\widehat p_a\propto -T_{\mathrm{sat}},
$$

then the result is inconsistent with the narrow hypothesis of constant air molar concentration and temperature tracking $T_{\mathrm{sat}}$, for which $p_a=c_aRT$ gives a positive slope.

It does **not** prove that no noncondensables are present. It can also result from:

- changing air concentration in an open system;
- changing leakage/removal balance;
- subcooling correlated with circulating-water temperature;
- hotwell drain mixing;
- a biased or nonrepresentative temperature measurement;
- changing spatial air distribution; or
- covariance with load and other operating conditions.

In the special hypothetical case where a co-located, calibrated temperature element is known to measure the equilibrium steam saturation temperature and air molar concentration is independently known to be constant, the residual calculation is valid and $p_a$ should increase approximately linearly with absolute temperature. A persistent negative relationship would then disprove at least one of those assumptions. If the residual were statistically indistinguishable from zero after measurement uncertainty was propagated, it would support a conclusion of no **significant detectable** noncondensables under those conditions.

### 6.2 Why offset magnitude still matters

The physical inequality $p_s<P_{\mathrm{cond}}$ is necessary, but many possible offsets can enforce it. Because water saturation pressure is nonlinear in temperature, the chosen offset changes both the magnitude and time dependence of

$$
P_{\mathrm{sat}}(T+T_{\mathrm{offset}})
$$

and therefore can change the inferred pressure residual and its correlation. A constant offset should be presented as an explicit sensitivity assumption, not as calibration proof.

### 6.3 Industry precedent and a defensible conclusion

Industry does use an **excess pressure** concept based on measured condenser pressure and a saturation pressure inferred from a representative condensate or vapor temperature. EPRI notes that a step increase in backpressure at constant load, circulating-water temperature and flow, hotwell temperature, and normal exhauster capacity indicates increased air partial pressure.

Industry practice does not normally diagnose air using correlation sign alone. It combines:

- corrected condenser backpressure or effective $U$;
- hotwell temperature and subcooling;
- circulating-water inlet/outlet temperature and flow;
- terminal temperature difference;
- measured off-gas or air-removal flow;
- vacuum-pump or steam-jet-air-ejector performance;
- dissolved oxygen; and
- tracer-gas leak testing.

A credible conclusion from the available proxy analysis is:

> The measurements do not show the positive pressure-temperature signature predicted by a constant-concentration lumped-air model. Because the required vapor equilibrium temperature is not directly measured and the condenser is an open, spatially nonuniform system, the analysis does not exclude air in-leakage. It instead shows that this particular inferred-partial-pressure model is not supported without additional temperature and air-removal information.

## 7. Experimental air-property evidence

### 7.1 Waxman and Davis

Waxman and Davis measured dry, ultra-pure air at 298.15 K with the Burnett method. The gas was confined in calibrated apparatus and subjected to stepwise isothermal expansions. Their reported low-pressure values include:

| Pressure | Molar density | Compressibility factor |
|---:|---:|---:|
| 0.01 MPa | 4.0341 mol/m3 | 0.99997 |
| 0.05 MPa | 20.1729 mol/m3 | 0.99985 |
| 0.10 MPa | 40.3519 mol/m3 | 0.99970 |

The lowest point, 0.01 MPa, is approximately 2.95 inHgA and is directly relevant to a typical steam condenser.

### 7.2 NIST reference equation

Lemmon et al. developed a Helmholtz-energy equation of state for standard dry air using evaluated experimental PVT, heat-capacity, speed-of-sound, virial-coefficient, and phase-equilibrium data. NIST reports approximately 0.1% density uncertainty across a range far broader than the present low-pressure service.

These sources establish that the dry-air state relation is well known. They do not determine condenser air inventory or its variation with operating conditions.

## 8. Air blanketing and heat-transfer reduction

### 8.1 Henderson-Marchello correlation used by EPRI

EPRI reproduces the Henderson-Marchello experimental correlation for the ratio of steam-side condensation film coefficient with air to the corresponding air-free Nusselt value:

$$
\boxed{
f_s=\frac{h_{s,a}}{h_{s,0}}
=\frac{1}{1+0.51Y_a}
}
$$

where $Y_a$ is **mole percent air**, not mole fraction.

The correlation is steep at low concentration:

| Air, mol% | $h_{s,a}/h_{s,0}$ | Steam-side reduction |
|---:|---:|---:|
| 0.10% | 0.951 | 4.9% |
| 0.261% | 0.883 | 11.7% |
| 0.50% | 0.797 | 20.3% |
| 1.00% | 0.662 | 33.8% |
| 2.00% | 0.495 | 50.5% |
| 5.00% | 0.282 | 71.8% |

This is a steam-side film-coefficient correction, not automatically an overall-condenser $U$ correction.

### 8.2 Example: 0.01 mol/m3 at 3 inHgA

Assume:

$$
P_{\mathrm{cond}}=3\ \mathrm{inHgA}
=10{,}159\ \mathrm{Pa}
$$

and $T_a\approx319\ \mathrm{K}$, close to water saturation temperature at this pressure.

Then

$$
p_a=c_aRT_a
=(0.01)(8.314)(319)
=26.5\ \mathrm{Pa},
$$

and

$$
Y_a
=100\frac{p_a}{P_{\mathrm{cond}}}
=0.261\%.
$$

The Henderson-Marchello factor is

$$
f_s=\frac{1}{1+0.51(0.261)}=0.883.
$$

Thus the estimated steam-side film-coefficient reduction is

$$
\boxed{11.7\%}.
$$

### 8.3 Converting steam-side loss to overall $U$

On a consistent area basis,

$$
\frac{1}{U_0}
=R_s+R_{\mathrm{tube}}+R_{\mathrm{water}}+R_{\mathrm{fouling}}.
$$

Define the baseline steam-side fraction of total resistance as

$$
\phi_s=\frac{R_s}{R_{\mathrm{total}}}
=\frac{U_0}{h_{s,0}}.
$$

If air multiplies $h_s$ by $f_s$, then

$$
\boxed{
\frac{U_a}{U_0}
=\frac{1}
{1+\phi_s\left(\frac{1}{f_s}-1\right)}
}.
$$

For $f_s=0.883$:

| Steam-side resistance fraction $\phi_s$ | Overall $U$ reduction |
|---:|---:|
| 0.20 | 2.6% |
| 0.30 | 3.8% |
| 0.40 | 5.1% |
| 0.50 | 6.2% |

Without the condenser's actual tube-side, wall, fouling, and clean steam-side resistances, the whole-condenser reduction is not uniquely identifiable. A nominal 4% reduction with approximately 2-6% sensitivity is reasonable for a lumped MML study, but it is not a plant-specific prediction.

### 8.4 Spatial-distribution limitation

Air is not generally distributed uniformly over every tube. Condensation scavenges noncondensables toward the interior and air-removal section. Some tubes may remain steam rich while a smaller region becomes severely air bound.

EPRI explicitly cautions that Henderson factors are difficult to apply confidently to an operating condenser because the location and extent of air binding are seldom known, even if incoming air concentration is known. NRC condensation work likewise treats the local air-rich diffusion layer as the governing mechanism.

Consequently, the same volume-average concentration can cause very different whole-bundle $U$ reductions depending on bundle geometry and air-removal performance.

## 9. When is air concentration problematic?

There is no universal problematic concentration in mol/m3 because:

- concentration is strongly nonuniform;
- plant impact depends on condenser pressure and temperature;
- the air-removal section intentionally concentrates air;
- local affected area matters as much as bulk concentration; and
- turbine and condenser performance limits are plant specific.

For orientation, at 3 inHgA and 319 K:

| Air concentration | Approx. air mol% | Henderson steam-side reduction |
|---:|---:|---:|
| 0.001 mol/m3 | 0.026% | 1.3% |
| 0.005 mol/m3 | 0.131% | 6.2% |
| 0.010 mol/m3 | 0.261% | 11.7% |
| 0.020 mol/m3 | 0.522% | 21.0% |
| 0.040 mol/m3 | 1.044% | 34.7% |
| 0.100 mol/m3 | 2.610% | 57.1% |

These are local or uniformly mixed shell-side estimates, not guaranteed whole-condenser losses. EPRI notes that approximately 1 mol% air can already have a substantial shell-side effect.

## 10. Evaluation of the MML `air_intake` fault

Relevant files:

- [`Condenser.mo`](../MetroscopeModelingLibrary/WaterSteam/HeatExchangers/Condenser.mo)
- [`SteamTurbine.mo`](../MetroscopeModelingLibrary/WaterSteam/Machines/SteamTurbine.mo)
- [`Condenser_faulty.mo`](../MetroscopeModelingLibrary/Tests/WaterSteam/HeatExchangers/Condenser_faulty.mo)

### 10.1 What the model does correctly

The condenser computes

$$
P_{\mathrm{incond}}
=P_{\mathrm{offset}}
+R(C_{\mathrm{incond}}+\texttt{air_intake})T_{\mathrm{sat}}.
$$

It subtracts that pressure before evaluating pure-water condensation properties and restores it afterward. The connected turbine therefore sees

$$
P_{\mathrm{out,turbine}}
=P_s+P_{\mathrm{incond}}.
$$

The turbine uses its outlet pressure in both Stodola's flow relationship and its isentropic outlet-state calculation. This correctly captures the direct mechanical-backpressure mechanism:

- higher air partial pressure raises turbine exhaust pressure;
- specific turbine work decreases; and
- flow capacity can also decrease when upstream conditions are fixed.

The isenthalpic pressure transformation inside the condenser should be understood as a bookkeeping approximation for changing from a pure-water turbine state referenced to total mechanical pressure to a steam-component state referenced to partial pressure after mixing. It is not a literal throttling valve.

### 10.2 Principal limitations

1. **`air_intake` is not an intake rate.** Its units are mol/m3, so it represents an imposed excess molar concentration.

2. **No air inventory balance exists.** There is no condenser gas volume, leak flow, dissolution term, vacuum-pump/ejector removal curve, or transient accumulation.

3. **Air temperature is assumed equal to `Tsat`.** The air-removal region can be substantially colder than the bulk steam region.

4. **The model is lumped.** It does not represent local air enrichment, the air cooler, diffusion through an air boundary layer, or air-bound tube area.

5. **The direct heat-transfer penalty is missing.** The condenser heat-transfer expression uses `Kth*(1-fouling/100)` but contains no air-dependent correction. At fixed duty inputs and cooling-water conditions, `air_intake` can therefore add backpressure without directly forcing steam partial pressure or condensate saturation temperature upward.

6. **`P_offset` is calibration rather than physics.** A fitted offset can be useful, but the model should assert nonnegative air pressure and document the fitted operating range.

7. **The provided faulty component test ramps several faults simultaneously.** `Condenser_faulty.mo` ramps fouling, air concentration, and cooling-water-flow decrease together, so it is not an isolated validation of the air fault.

### 10.3 Overall assessment

The current MML fault is physically coherent as a reduced-order **excess noncondensable concentration/backpressure fault**. It is not a predictive air-in-leakage model. In particular, it will generally underrepresent the rise in steam partial pressure and hotwell temperature caused by air-related heat-transfer degradation.

## 11. Recommended MML formulation

### 11.1 Minimum algebraic improvement

If the imposed-concentration structure is retained, rename or document the fault as `excess_air_molar_concentration` and calculate:

$$
p_a=Zc_aRT_a,
\qquad
Y_a=100\frac{p_a}{P_{\mathrm{cond}}},
$$

$$
f_s=\frac{1}{1+0.51Y_a},
$$

and

$$
f_U
=\frac{1}
{1+\phi_s\left(\frac{1}{f_s}-1\right)}.
$$

Then use

$$
K_{\mathrm{th,eff}}
=K_{\mathrm{th,clean}}
\left(1-\frac{\mathrm{fouling}}{100}\right)f_U.
$$

For 0.01 mol/m3 at 3 inHgA, a nominal whole-condenser implementation would use approximately

$$
K_{\mathrm{th,eff}}\approx0.96K_{\mathrm{th,clean}}
$$

for the air effect alone, with sensitivity of roughly 0.94-0.98 depending on $\phi_s$.

If `Kth` is instead intended to represent only the steam-side coefficient, use $f_s=0.883$, not 0.96.

### 11.2 Preferred mechanistic improvement

For a predictive fault model, make leakage a molar flow and solve an air balance:

$$
\frac{d(n_a)}{dt}
=\dot n_{a,\mathrm{leak}}
-\dot n_{a,\mathrm{removal}}(p_a,T_a,\text{equipment state})
-\dot n_{a,\mathrm{dissolution}}.
$$

Then calculate

$$
p_a=Z\frac{n_a}{V_g}RT_a,
\qquad
P_{\mathrm{cond}}=p_s+p_a,
$$

and couple the air concentration to condensation resistance or affected tube area. The steam partial pressure and saturation temperature should emerge from the coupled turbine, mass, and heat balances rather than being forced upward explicitly.

## 12. Suggested reporting language for a plant

> Condenser total pressure equals the sum of steam and noncondensable partial pressures. The turbine responds mechanically to total exhaust pressure, while condensation equilibrium depends on steam partial pressure. We estimated noncondensable pressure by subtracting saturation pressure calculated from a condensate-temperature proxy. Because the measured condensate temperature is not demonstrably the representative vapor equilibrium temperature and exceeds the total-pressure saturation temperature during part of the period, an explicit temperature-offset sensitivity was required. The resulting negative pressure-temperature relationship rejects a constant-concentration lumped-air hypothesis but does not, by itself, exclude air in-leakage. Confirmation should incorporate corrected condenser performance, circulating-water conditions, air-removal flow and equipment status, dissolved oxygen, and leak testing. If air concentration is modeled, its direct addition to turbine backpressure and its reduction of steam-side condensation heat transfer should both be represented.

## 13. References

1. Electric Power Research Institute (EPRI), *Condenser In-Leakage Guideline*, TR-112819. Includes condenser-performance methods, air-removal guidance, the Henderson-Marchello curve in Figure 2-3, and cautions concerning localized air binding. https://restservice.epri.com/publicdownload/TR-112819/0/Product

2. Heat Exchange Institute (HEI), *Tech Sheet #113: Condenser Basics*. Explains that noncondensables add partial pressure, reduce turbine output and efficiency, and blanket tubes. https://www.heatexchange.org/wp-content/uploads/2025/04/FINAL_Tech_Sheet_113_Condenser_Basics-2c4a0c84.pdf

3. C. L. Henderson and J. M. Marchello, "Film Condensation in the Presence of a Noncondensable Gas," *Journal of Heat Transfer*, 91(3), 447-450, 1969. https://doi.org/10.1115/1.3580218

4. J. Lee, C.-J. Choi, H. K. Cho, et al., *Assessment of the Wall Film Condensation Model with Non-condensable Gas in RELAP5 and TRACE for Vertical Tube and Plate Geometries*, NRC NUREG/IA-0491, 2019. Reviews diffusion-layer modeling and comparison against seven condensation experiments. https://www.nrc.gov/reading-rm/doc-collections/nuregs/agreement/ia0491/index

5. M. Waxman and H. Davis, "Density of Ultra-Pure Air at 298.15 K for Mass Transfer Buoyancy Corrections," *Journal of Research of the National Bureau of Standards*, 83(5), 415-418, 1978. https://doi.org/10.6028/jres.083.027

6. E. W. Lemmon, R. T. Jacobsen, S. G. Penoncello, and D. G. Friend, "Thermodynamic Properties of Air and Mixtures of Nitrogen, Argon, and Oxygen From 60 to 2000 K at Pressures to 2000 MPa," *Journal of Physical and Chemical Reference Data*, 29(3), 331-385, 2000. https://doi.org/10.1063/1.1285884 and https://www.nist.gov/publications/thermodynamic-properties-air-and-mixtures-nitrogen-argon-and-oxygen-60-2000-k-pressures

7. NIST, *REFPROP - Reference Fluid Thermodynamic and Transport Properties Database*. Provides evaluated equations of state and derivatives for air and other fluids. https://www.nist.gov/srd/refprop

## 14. Quick-use prompt for Codex or ChatGPT

Upload or reference this Markdown file and use a prompt such as:

> Use `condenser_air_inleakage_technical_reference.md` as the governing technical context. Clearly distinguish total condenser static pressure, steam partial pressure, air inventory, and hotwell temperature. State which assumptions are needed for any inference. For heat-transfer calculations, distinguish the steam-side Henderson-Marchello factor from the overall condenser U factor. When reviewing MML, trace both the condenser pressure decomposition and the connected turbine response.
