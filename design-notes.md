Resources
---------

[FOC introduction](https://www.roboteq.com/technology/field-oriented-control)

[Brushless FOC info](http://ww1.microchip.com/downloads/en/AppNotes/01078B.pdf)

[Brushless motor driver considerations](https://www.ti.com/lit/an/slyt692/slyt692.pdf?ts=1634725803450&ref_url=https%253A%252F%252Fwww.google.com%252F)

[Trapezoidal example](https://www.cypress.com/file/71631/download)

[Current sensing](https://www.st.com/resource/en/application_note/dm00666970-current-sensing-in-bldc-motor-application-stmicroelectronics.pdf)

[Low side current sensing](https://www.nxp.com/docs/en/brochure/BB3PHCRMSRART.pdf)

Principle operation
-------------------

A sensorless FOC inverter works as follows:

-   The controller measures the current and voltage on each phase of the
    motor.

-   The current and voltage measurements are combined to estimate the
    back-emf. Since the measured voltage is the sum of the back-emf and
    other components, the current (and rate of change of current) must
    be used to extract the back-emf.

-   The back-emf measurements are combined to calculate the rotor angle.

-   The objective is now to control the currents of each phase in order
    to produce a magnetic field with components parallel and
    perpendicular to the rotor angle. The parallel (known as the
    \"flux\" or D) component is typically made to be 0, since this
    component doesn\'t apply any torque on the rotor. The perpendicular
    component (known as the \"torque\" or Q) component determines the
    torque applied on the rotor. This may be set by some higher-level
    control loop in order to regulate, for example, the speed.

-   Using a model of a motor consisting of a resistor, inductor and
    time-varying voltage source in series, the phase voltages required
    in order to change the current components can be calculated.

-   The PWM waveforms of each phase are calculated using Space Vector
    Modulation, which switches the FETs in such a way that the relative
    average voltages between all phases can be controlled.

Schematic considerations
------------------------

### Measurements

1.  Sensing phase current

    To precisely control the magnetic field contribution of each phase,
    the current of each phase must be known. There are different methods
    of measuring the phase current:

    -   Low side sensing, which uses a shunt resistor between the source
        of each low-side FET and ground.

    -   Inline sensing, which uses a shunt resistor directly in series
        with each phase.

    -   High side sensing, which uses a shunt resistor between the drain
        of each high-side FET and the positive DC rail.

    When using low side sensing, the current can only be measured while
    the relevant low-side FET is turned on. When the low-side FET is
    off, the current for that phase goes through the high-side FET
    instead. The measurement must be taken a short time period after the
    low-side FET turns on, to give it time to stabilise. This can make
    it difficult to accurately measure the current when the PWM for that
    phase is close to 100%.

    Inline sensing solves this problem by measuring the phase currents
    directly. However, it is more difficult to accurately read this
    current: since the phase voltages change very quickly each time the
    FETs switch, the amplifier must have high common-mode rejection.

    High side sensing is similar to low side sensing, since the
    measurement must be taken at a specific point. This time, however,
    the high-side FET must be on, meaning measurements are difficult
    when the PWM is near 0%. Note that low PWM doesn\'t mean low power -
    if all three phases have their pulses aligned (at any PWM), then no
    current will flow through the motor. High side sensing also has the
    added difficulty of finding an amplifier that will work close to the
    supply rails.

    Remember that all three options involve bidirectional flow of
    current through the shunt resistor. Due to its inductance, the
    current going into a phase remains roughly constant over one PWM
    cycle. This means that for a current *into* the phase, the bottom
    FET will be sourcing current when it is turned on. Similarly for a
    current *out of* the phase, the top FET will be sinking current when
    it is turned on. This needs to be taken into consideration when
    measuring the currents.

    Also bear in mind that only two phases need to be measured, since
    the sum of the three phase currents must be 0. This allows the third
    to be calculated without direct measurement.

2.  Sensing DC current

    It is not necessary to measure the total DC current to control the
    motor. However, it may be useful for implementing hardware
    overcurrent protection.

3.  Estimating back-emf

    In a sensorless inverter,the back-emf is used to calculate the rotor
    angle. There are several ways that this can be done:

    -   Measurement of phase voltage on a \"floating\" phase.

    -   Using the phase current to estimate the back-emf.

    Here we will model each phase of the motor as some resistance, some
    inductance and a time-varying voltage source in series. The voltage
    on phase $n$ is therefore:

    $V_n(t) = R i_n(t) + L \frac{d}{dt} i_n(t) + e_n(t) + V_{neutral}(t)$

    Where $V_n(t)$ is the terminal voltage, $R$ is the phase resistance,
    $L$ is the phase inductance, $e_n(t)$ is the phase emf, and
    $V_{neutral}(t)$ is the voltage of the neutral point in the motor
    (where the three phases are connected).

    The floating phase measurement works by turning off both high and
    low side FETs, leaving the phase A to \"float\". Once the current
    into phase A has reached 0 (which will take a small delay, due to
    the phase inductance), we know that the currents (and the rate of
    change of currents) into phases B and C are equal and opposite.
    Therefore:

    $V_b(t) + V_c(t) = R (i_b(t) + i_c(t)) + L (\frac{d}{dt}
         i_b(t) + \frac{d}{dt} i_c(t)) + e_b(t) + e_c(t) + 2
         V_{neutral}(t) = e_b(t) + e_c(t) + 2 V_{neutral}(t)$

    We also know:

    $V_a(t) = e_a(t) + V_{neutral}(t)$

    Therefore:

    $V_b(t) + V_c(t) - 2V_a(t) = e_b(t) + e_c(t) - 2e_a(t) =
         (e_a(t) + e_b(t) + e_c(t)) - 3e_a(t)$

    But the sum of the emfs will be zero, so:

    $V_b(t) + V_c(t) - 2V_a(t) = 3e_a(t)$

    $e_a(t) = \frac{V_b(t) + V_c(t) - 2V_a(t)}{3}$

    Note that it isn\'t strictly necessary to directly measure $V_b$ and
    $V_c$ when calculating $e_a$, since phases B and C will either be
    pulled high or pulled low, meaning their terminal voltages are
    known. However, this assumes that the measurement of $V_a$ is taken
    when neither phase is switching, meaning careful timing is required.

    Since the phase must be made to float for sufficiently long that the
    phase current can fall to zero and a measurement can be taken, too
    frequent voltage measurements can disrupt the operation of the motor
    driver. This can be minimised by choosing to take measurements when
    the phase current is close to 0, which reduces both the time for the
    inductance to discharge, and the power lost while the measurement is
    being taken.

4.  Sensing DC voltage

### Shunt resistor considerations

1.  Power dissipation

2.  Filtering

3.  Amplification

### Parameter considerations

1.  PWM frequency

2.  Maximum frequency

    Different to motor speed.

3.  Maximum current

4.  Maximum voltage

### Control method

1.  Trapezoidal

2.  Sinusoidal

3.  Field Oriented Control (FOC)

### Processor considerations

1.  PWM support

2.  Floating point support

3.  ADC precision

4.  ADC timing

5.  Processor speed

### Transistor considerations

1.  Maximum source-drain voltage

2.  Maximum current

3.  Source-drain on-resistance

4.  Gate-source voltage

5.  Gate charge

### Gate driver considerations

1.  Topology

2.  Current

3.  Gate current limiting

### Other considerations

1.  DC bus capacitance

PCB considerations
------------------

### Number of layers

### Analogue signals

Software considerations
-----------------------

### Control loop frequency

### 
