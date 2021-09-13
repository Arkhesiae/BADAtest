export class PhysicalPlane {
    flightCoefficients = {
        acTypeData: {
            engineType: "JET",
            numberOfEngines: "2",
            wakeTurbulenceCategory: "MEDIUM"
        },
        aerodynamics: {
            buffetCoefficient: "1.2273",
            buffetGradient: "0.47037",
            inducedDragApproach: "0.035779",
            inducedDragCruise: "0.025882",
            inducedDragLanding: "0.036689",
            machDragCoefficient: "0.0",
            parasiticDragApproach: "0.046986",
            parasiticDragCruise: "0.025954",
            parasiticDragLanding: "0.097256",
            parasiticDragLandingGear: "0.02568",
            stallSpeedApproaching: "1000",
            stallSpeedCruise: "1390",
            stallSpeedInitialClimbing: "1130",
            stallSpeedLanding: "940",
            stallSpeedTakeoff: "1040",
            wingSurfaceArea: "122.6"
        },
        aircraftStandardMass: {
            maximumMass: "70000",
            maximumPayloadMass: "17000",
            minimumMass: "40000",
            referenceMass: "60000"
        },
        aircraftEngineThrust: {
            firstMaxClimbThrust: "140720.0",
            thirdMaxClimbThrust: "9.6625E-11",
            approachThrust: "0.14767",
            descentMach: "0.78",
            descentSpeed: "3000",
            firstThrustTemperatureCoefficient: "9",
            highAltitudeDescentThrust: "0.083084",
            landingThrust: "0.34217",
            lowAltitudeDescentThrust: "0.051765",
            secondMaxClimbThrust: "47489.0",
            secondThrustTemperatureCoefficient: "0.0094754",
            transitionAltitude: "27726.0"
        },
        aircraftFlightEnvelope: {
            altitudeTemperatureGradient: "-160",
            maxAltitudeAtMTOW: "37575",
            maxOperatingAltitude: "41000",
            maxOperatingMachNumber: "0.82",
            maxOperatingSpeed: "3500",
            weightGradient: "0.31247"
        },
        groundPerformance: {
            landingLength: "1470.0",
            length: "33.84",
            takeoffLength: "1820.0",
            wingSpan: "34.1"
        },
        aircraftFuelConsumption: {
            firstThrustConsumption: {
                JET: {
                    jetConsumption: "0.72891"
                }
            },
            cruiseConsumption: "0.99224",
            firstDescentConsumption: "11.114",
            secondDescentConsumption: "133850.0",
            secondThrustConsumption: "1729.8"
        },
        aircraftType: "A319"
    }

    dVTAS = 0

    flightParams = {
        speed: {
            CAS: "",
            TAS: "",
            Mach: "",
        },

        Hp: "",
        ROCD: "",
        mass: ""
    }

    force = {
        drag: "",
        thrust: "",
    }

    atmosphereParams = {
        pressure: "",
        deltaT: "",
        temperature: "",
        airDensity: "",
    }

    loiMontee = {
        CAS: "",
        Mach: "",
        HpTrans: "",
    }

    maxThrust = 0

    constructor() {
    }

    setParameters(mass) {
        this.flightParams.mass = mass
    }

    setLoi(CAS, M) {
        this.loiMontee.CAS = CAS
        this.loiMontee.Mach = M
        this.loiMontee.HpTrans = HpTrans(knotToMs(CAS), M)
    }

    setInitialState(altitude, speed, ROCD, deltaT) {
        this.flightParams.Hp = altitude
        this.flightParams.speed.CAS = speed
        this.atmosphereParams.deltaT = deltaT
        this.setAtmosphere()
        this.flightParams.speed.TAS = CAStoTAS(this.flightParams.speed.CAS, this.atmosphereParams.pressure, this.atmosphereParams.temperature)
        this.computeSpeed()
        this.computeAerodynamics()
        this.flightParams.ROCD = ROCD

    }

    setAtmosphere() {
        this.atmosphereParams.temperature = temperature(this.atmosphereParams.deltaT, this.flightParams.Hp)
        // console.log(temperature(this.atmosphereParams.deltaT, 11000))
        this.atmosphereParams.pressure = airPressure(this.atmosphereParams.deltaT, this.flightParams.Hp)
        this.atmosphereParams.airDensity = airDensity(this.atmosphereParams.pressure, this.atmosphereParams.temperature)
        let Ctc1 = parseFloat(this.flightCoefficients.aircraftEngineThrust.firstMaxClimbThrust)
        let Ctc2 = parseFloat(this.flightCoefficients.aircraftEngineThrust.secondMaxClimbThrust)
        let Ctc3 = parseFloat(this.flightCoefficients.aircraftEngineThrust.thirdMaxClimbThrust)
        this.maxThrust = parseFloat(maxClimbTOThrust(Ctc1, Ctc2, Ctc3, this.flightParams.Hp))
    }

    computeSpeed() {
        this.flightParams.speed.CAS = TAStoCAS(this.flightParams.speed.TAS, this.atmosphereParams.pressure, this.atmosphereParams.temperature)
        // console.log("TAS", msToKnot(CAStoTAS(this.flightParams.speed.CAS, this.atmosphereParams.pressure, this.atmosphereParams.temperature)))
        this.lowSpeedBuffetingLimit = lowSpeedBuffetting(parseFloat( this.flightCoefficients.aerodynamics.buffetGradient), parseFloat( this.flightCoefficients.aerodynamics.buffetCoefficient),  this.atmosphereParams.pressure, parseFloat( this.flightCoefficients.aerodynamics.wingSurfaceArea), this.flightParams.mass*Constante.g0)
        this.maxMach = 0.82
        this.flightParams.speed.Mach = TAStoMach(this.flightParams.speed.TAS, this.atmosphereParams.temperature)
        // console.log(CAStoTAS(55, this.atmosphereParams.pressure, this.atmosphereParams.temperature))
    }

    computeAerodynamics() {
        this.force.drag = drag(this.atmosphereParams.airDensity, this.flightParams.speed.TAS, parseFloat(this.flightCoefficients.aerodynamics.wingSurfaceArea), 0, parseFloat(this.flightCoefficients.aerodynamics.parasiticDragCruise), parseFloat(this.flightCoefficients.aerodynamics.inducedDragCruise), this.flightParams.mass)
    }

    computeThrust() {
        let Ctc1 = parseFloat(this.flightCoefficients.aircraftEngineThrust.firstMaxClimbThrust)
        let Ctc2 = parseFloat(this.flightCoefficients.aircraftEngineThrust.secondMaxClimbThrust)
        let Ctc3 = parseFloat(this.flightCoefficients.aircraftEngineThrust.thirdMaxClimbThrust)
        this.maxThrust = parseFloat(maxClimbTOThrust(Ctc1, Ctc2, Ctc3, this.flightParams.Hp))
        this.force.thrust = maxClimbTOThrust(Ctc1, Ctc2, Ctc3, this.flightParams.Hp)
    }

    computeMaxClimbThrust() {
        let Ctc1 = parseFloat(this.flightCoefficients.aircraftEngineThrust.firstMaxClimbThrust)
        let Ctc2 = parseFloat(this.flightCoefficients.aircraftEngineThrust.secondMaxClimbThrust)
        let Ctc3 = parseFloat(this.flightCoefficients.aircraftEngineThrust.thirdMaxClimbThrust)
        this.maxThrust = parseFloat(maxClimbTOThrust(Ctc1, Ctc2, Ctc3, this.flightParams.Hp))
    }

    monteeCASConstant() {
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        this.computeThrust()
        let ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, this.atmosphereParams.deltaT, this.flightParams.Hp, "constant", 0)
        this.flightParams.ROCD = ROCD(this.atmosphereParams.temperature, this.force.thrust, this.force.drag, this.flightParams.speed.TAS, ESFValue, this.flightParams.mass, this.atmosphereParams.deltaT)
        let dVTAS = (1 / ESFValue - 1) * Constante.g0 / this.flightParams.speed.TAS * this.flightParams.ROCD * (this.atmosphereParams.temperature / (this.atmosphereParams.temperature - this.atmosphereParams.deltaT))
        this.flightParams.speed.TAS += dVTAS
        this.updateFlightParams()
    }

    palier(){
        this.computeThrustForPallier(this.flightParams.speed.TAS)
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        let ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, this.atmosphereParams.deltaT, this.flightParams.Hp, "constant", 0)
        this.flightParams.ROCD = ROCD(this.atmosphereParams.temperature, this.force.thrust, this.force.drag, this.flightParams.speed.TAS, ESFValue, this.flightParams.mass, this.atmosphereParams.deltaT)
        let dVTAS = (this.force.thrust - this.force.drag) / this.flightParams.mass
        this.flightParams.speed.TAS += dVTAS
        this.updateFlightParams()
    }

    update(){
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
    }

    desCASConstant() {
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        this.force.thrust = 0
        let ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "constant", 0)
        this.flightParams.ROCD = ROCD(this.atmosphereParams.temperature, this.force.thrust, this.force.drag, this.flightParams.speed.TAS, ESFValue, this.flightParams.mass, 0)
        // console.log(this.flightParams.ROCD*196)
        let dVTAS = (1 / ESFValue - 1) * Constante.g0 / this.flightParams.speed.TAS * this.flightParams.ROCD
        this.flightParams.speed.TAS += dVTAS
        this.updateFlightParams()
    }

    monteeDecel() {
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        this.computeThrust()
        let ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "decel", 1)
        this.flightParams.ROCD = ROCD(this.atmosphereParams.temperature, this.force.thrust, this.force.drag, this.flightParams.speed.TAS, ESFValue, this.flightParams.mass, 0)
        let dVTAS = (1 / ESFValue - 1) * Constante.g0 / this.flightParams.speed.TAS * this.flightParams.ROCD
        this.flightParams.speed.TAS += dVTAS
        this.updateFlightParams()
    }

    monteeAccel() {
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        this.computeThrust()
        let ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "accel", 1)
        this.flightParams.ROCD = ROCD(this.atmosphereParams.temperature, this.force.thrust, this.force.drag, this.flightParams.speed.TAS, ESFValue, this.flightParams.mass, 0)
        let dVTAS = (1 / ESFValue - 1) * Constante.g0 / this.flightParams.speed.TAS * this.flightParams.ROCD
        this.flightParams.speed.TAS += dVTAS
        this.updateFlightParams()
    }

    descentDecel() {
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        this.force.thrust = 0
        let ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "decel", -1)
        this.flightParams.ROCD = ROCD(this.atmosphereParams.temperature, this.force.thrust, this.force.drag, this.flightParams.speed.TAS, ESFValue, this.flightParams.mass, 0)
        let dVTAS = (1 / ESFValue - 1) * Constante.g0 / this.flightParams.speed.TAS * this.flightParams.ROCD
        this.flightParams.speed.TAS += dVTAS
        console.log(this)
        this.updateFlightParams()
    }

    descentAccel() {
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        this.force.thrust = 0
        let ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "accel", -1)
        this.flightParams.ROCD = ROCD(this.atmosphereParams.temperature, this.force.thrust, this.force.drag, this.flightParams.speed.TAS, ESFValue, this.flightParams.mass, 0)
        let dVTAS = (1 / ESFValue - 1) * Constante.g0 / this.flightParams.speed.TAS * this.flightParams.ROCD
        this.flightParams.speed.TAS += dVTAS
        this.updateFlightParams()
    }

    computeThrustForPallier(target) {
        let thrust = drag(this.atmosphereParams.airDensity, target, parseFloat(this.flightCoefficients.aerodynamics.wingSurfaceArea), 0, parseFloat(this.flightCoefficients.aerodynamics.parasiticDragCruise), parseFloat(this.flightCoefficients.aerodynamics.inducedDragCruise), this.flightParams.mass)
        this.force.thrust = thrust
    }

    accel() {
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        let target = 300
        if (Math.abs(this.flightParams.speed.TAS - knotToMs(target)) < 1) {
            this.computeThrustForPallier(knotToMs(target))
        } else {
            this.computeThrust()
        }
        this.flightParams.ROCD = 0
        let dVTAS = (this.force.thrust - this.force.drag) / this.flightParams.mass
        this.flightParams.speed.TAS += dVTAS
        this.updateFlightParams()
    }

    decel() {
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        let target = 380
        if (Math.abs(this.flightParams.speed.TAS - knotToMs(target)) < 1) {
            this.computeThrustForPallier(knotToMs(target))
        } else {
            this.computeThrust()
        }
        this.flightParams.ROCD = 0
        let dVTAS = (this.force.thrust - this.force.drag) / this.flightParams.mass
        this.flightParams.speed.TAS += dVTAS
        this.updateFlightParams()
    }

    climbAtROCD(targetROCD) {
        this.updateConditions()

        let thrustToReachTarget = this.force.drag + targetROCD / this.flightParams.speed.TAS * this.flightParams.mass * Constante.g0 + this.dVTAS * this.flightParams.mass
        this.force.thrust = Math.max(Math.min(thrustToReachTarget, this.maxThrust), 0)
        let ESFValue

        console.log(thrustToReachTarget, this.maxThrust)

        if (thrustToReachTarget >= this.maxThrust) {
            // Thrust et Vitesse limitantes
            if (Math.abs(this.flightParams.speed.TAS - CAStoTAS(80, this.atmosphereParams.pressure, this.atmosphereParams.temperature)) < 2) {
                console.log("STALL LIMIT")
                ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "constant", 1)
                this.flightParams.ROCD = ROCD(this.atmosphereParams.temperature, this.force.thrust, this.force.drag, this.flightParams.speed.TAS, ESFValue, this.flightParams.mass, 0)
                let dVTAS = (1 / ESFValue - 1) * Constante.g0 / this.flightParams.speed.TAS * this.flightParams.ROCD
                this.dVTAS = dVTAS
                this.flightParams.speed.TAS += dVTAS
                this.flightParams.speed.TAS = Math.max(this.flightParams.speed.TAS, CAStoTAS(80, this.atmosphereParams.pressure, this.atmosphereParams.temperature))
            }

            // Thrust limitant, déceleration nécessaire
            else {
                console.log('CAS 2')
                let ROCD
                let dVTAS = (this.force.thrust - this.force.drag) / this.flightParams.mass - this.flightParams.ROCD * Constante.g0 / this.flightParams.speed.TAS
                this.dVTAS = dVTAS
                // ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "decel", 1)
                if (this.flightParams.ROCD ===0){
                   ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "decel", 1)
                    ROCD = targetROCD
                }
                else {
                    ROCD = ((this.force.thrust - this.force.drag)*this.flightParams.speed.TAS - this.flightParams.mass*this.flightParams.speed.TAS*this.dVTAS)/this.flightParams.mass/Constante.g0
                }
                // ESFValue =  (1+this.flightParams.speed.TAS/Constante.g0*this.dVTAS/this.flightParams.ROCD)**(-1)



                console.log(ROCD)

                this.flightParams.ROCD = ROCD
                this.flightParams.speed.TAS += dVTAS


                this.flightParams.speed.TAS = Math.max(this.flightParams.speed.TAS, CAStoTAS(80, this.atmosphereParams.pressure, this.atmosphereParams.temperature))
            }
            // console.log("maxThrust")
            // ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "decel", 1)
        } else {
            console.log("CAS 3")
            ESFValue = ESF(this.flightParams.speed.Mach, this.loiMontee.Mach, knotToMs(this.loiMontee.CAS), this.atmosphereParams.temperature, 0, this.flightParams.Hp, "constant", 1)
            this.flightParams.ROCD = ROCD( this.atmosphereParams.temperature, this.force.thrust, this.force.drag, this.flightParams.speed.TAS, ESFValue, this.flightParams.mass, 0)
            let dVTAS = (1 / ESFValue - 1) * Constante.g0 / this.flightParams.speed.TAS * this.flightParams.ROCD
            this.dVTAS = dVTAS
            this.flightParams.speed.TAS += dVTAS

        }
        this.updateFlightParams()
    }

    descentAtROCD(targetROCD) {

    }

    updateConditions(){
        this.setAtmosphere()
        this.computeSpeed()
        this.computeAerodynamics()
        this.computeMaxClimbThrust()
    }


    updateFlightParams() {
        this.flightParams.Hp += this.flightParams.ROCD
        this.flightParams.Hp = Math.max(this.flightParams.Hp, 0)
        if (this.flightParams.Hp===0){
            this.flightParams.ROCD =0
        }
    }
}

// Definitions

class Constante {
    // Standard atmospheric temperature at MSL
    static T0 = 288.15 //K
    // Standard atmospheric pressure at MSL
    static p0 = 101325 //Pa
    // Standard atmospheric density at MSL:
    static r0 = 1.225 //kg/m3
    // Speed of sound
    static a0 = 340.294 //m/s
    // Altitude Tropopause
    static HpTrop = 11000 //m
    // Adiabatic index of air
    static K = 1.4
    // Coefficient
    static mu = (this.K - 1) / this.K
    // Real gas constant for air
    static R = 287.05287 // m²/(K·s²)
    // Gravitational acceleration
    static g0 = 9.80665 // m/s²
    // ISA temperature gradient with altitude below the tropopause
    static Bt = -0.0065 // K/m
    // ISA temperature at tropopause
    static TISAtrop = this.T0 + this.Bt * this.HpTrop
}

export function msToKnot(speed) {
    return 1.9438 * speed
}

export function knotToMs(speed) {
    return speed / 1.9438
}

function airDensity(p, T) {
    return p / (Constante.R * T)
}

function speedOfSound(T) {
    return Math.sqrt(Constante.K * Constante.R * T)
}

function airPressure(deltaT, Hp) {
    let T = temperature(deltaT, Hp)
    let p = Constante.p0 * ((T - deltaT) / Constante.T0) ** (-Constante.g0 / (Constante.Bt * Constante.R))
    if (Hp <= Constante.HpTrop) {
        // console.log("Sous la tropopause : ", p)
        // Sous la tropopause
        // A la tropopause
        return p
    } else {
        // Au dessus de la tropopause
        return p * Math.exp(-Constante.g0 / (Constante.R * Constante.TISAtrop) * (Hp - Constante.HpTrop))
    }
}

function temperature(deltaT, Hp) {
    if (Hp <= Constante.HpTrop) {
        return Constante.T0 + deltaT + Constante.Bt * Hp
    } else {
        return Constante.TISAtrop + deltaT
    }

}

function CAStoTAS(CAS, p, T) {
    let r = airDensity(p, T)
    let A = (1 + Constante.mu / 2 * Constante.r0 / Constante.p0 * CAS ** 2) ** (1 / Constante.mu) - 1
    let B = (1 + Constante.p0 / p * A) ** Constante.mu - 1
    return Math.sqrt(2 / Constante.mu * p / r * B)
}

function TAStoCAS(TAS, p, T) {
    let r = airDensity(p, T)
    let A = (2 * Constante.p0 / Constante.mu / Constante.r0)
    let C = 1 + Constante.mu * r / p / 2 * TAS ** 2
    let B = 1 + (p / Constante.p0) * (C ** (1 / Constante.mu) - 1)
    return (A * (B ** Constante.mu - 1)) ** (1 / 2)
}

function MachtoTAS(M, T) {
    return M * Math.sqrt(Constante.K * Constante.R * T)
}

function TAStoMach(TAS, T) {
    return TAS / Math.sqrt(Constante.K * Constante.R * T)
}


/**
 * Determine l'altitude de conjonction
 * @param CAS Calibrated airspeed [m/s]
 * @param M Mach
 * @returns {number} Altitude Pression alti transition [ft]
 */
function HpTrans(CAS, M) {

    let A = (1 + (Constante.K - 1) / 2 * (CAS / Constante.a0) ** 2) ** (1 / Constante.mu) - 1
    let B = (1 + (Constante.K - 1) / 2 * M ** 2) ** (1 / Constante.mu) - 1
    let deltaTrans = A / B
    let DeltaTrans = deltaTrans ** (-Constante.Bt * Constante.R / Constante.g0)
    return 1000 / (0.3048 * 6.5) * Constante.T0 * (1 - DeltaTrans)
}

/**
 *
 * @param T
 * @param Thrust
 * @param Drag
 * @param TAS
 * @param ESF
 * @param m
 * @param deltaT
 * @returns {number}
 */
function ROCD(T, Thrust, Drag, TAS, ESF, m, deltaT) {
    return ((T - deltaT) / T) * ((Thrust - Drag) * TAS / m / Constante.g0) * ESF
}

function ESF(M, Mloi, CASloi, T, deltaT, Hp, State, Vertical) {
    switch (State) {
        case "constant":
            // CONSTANT SPEED
            if (Hp > HpTrans(CASloi, Mloi) / 3.28084) {
                //      Constant Mach Number
                //          Above Tropopause
                if (Hp > Constante.HpTrop) {
                    return 1
                }
                    //      Constant Mach Number
                //          Below Tropopause
                else {
                    return (1 + ((Constante.K * Constante.R * Constante.Bt) / (2 * Constante.g0)) * (M ** 2) * ((T - deltaT) / T)) ** (-1)
                }
            } else {
                let A
                let B = (1 + (Constante.K - 1) / 2 * M ** 2) ** (-1 / (Constante.K - 1))
                let C = (1 + (Constante.K - 1) / 2 * M ** 2) ** (1 / Constante.mu) - 1
                //      Constant CAS
                //          Above Tropopause
                if (Hp > Constante.HpTrop) {
                    A = 0
                }
                    //      Constant CAS
                //          Below Tropopause
                else {
                    console.log('hi')
                    A = 1 + Constante.K * Constante.R * Constante.Bt / (2 * Constante.g0) * M ** 2 * (T - deltaT) / T
                }
                return (A + B * C) ** (-1)
            }
        case "accel":
            // ACCELERATION
            if (Vertical > 0) {
                return 0.3
            } else {
                return 1.7
            }

        case "decel":
            // DECELERATION
            if (Vertical < 0) {
                return 0.3
            } else {
                return 1.7
            }
    }
}

function drag(rho, TAS, S, bank, CD0, CD2, m) {
    return CDragCR(CD0, CD2, m, rho, TAS, S, bank) * rho * TAS ** 2 * S / 2
}

function CLift(m, rho, TAS, S, bank) {
    return 2 * m * Constante.g0 / (rho * TAS ** 2 * S * Math.cos(bank * Math.PI / 180))
}

function CDragCR(CD0, CD2, m, rho, TAS, S, bank) {
    return CD0 + CD2 * CLift(m, rho, TAS, S, bank) ** 2
}

// MASS

function operatingSpeed(Vref, m, mRef) {
    return Vref * Math.sqrt(m / mRef)
}

// FLIGHT ENVELOPPE

/**
 * Fonction altitude maximale pour une masse donnée
 * @param hMo maximum operating altitude [ft] above standard MSL
 * @param hMax maximum altitude [ft] above standard MSL at MTOW under ISA
 conditions (allowing about 300 ft/min of residual rate of climb)
 * @param Gt temperature gradient on hmax [ft/K]
 * @param deltaT temperature deviation from ISA [K]
 * @param Ctc4
 * @param Gw mass gradient on hmax [ft/kg]
 * @param Mmax
 * @param Mact  actual aircraft mass [kg]
 * @returns {number}
 */
function maximumAltitude(hMo, hMax, Gt, deltaT, Ctc4, Gw, Mmax, Mact) {
    return Math.min(hMo, hMax + Gt * (deltaT - Ctc4) + Gw * (Mmax - Mact))
}

function minimumSpeed(CvMin, CvminTO, Vstall) {
    return CvMin * Vstall
}

function lowSpeedBuffetting(k, Clbo, p, S, W) {
    let a1 = - Clbo / k
    let a2 = 0
    let a3 = W / S / (0.583 * p * k)
    let Q = (3 * a2 - a1 ** 2) / 9
    let R = (9 * a1 * a2 - 27 * a3 - 2 * a1 ** 3) / 54
    let cosTheta = R / Math.sqrt(-(Q ** 3))
    let theta = Math.acos(cosTheta)

    let solutions = []
    let X1 = 2 * Math.sqrt(-Q) * Math.cos(theta / 3 ) - a1 / 3
    let X2 = 2 * Math.sqrt(-Q) * Math.cos(theta / 3 + 120 * Math.PI / 180) - a1 / 3
    let X3 = 2 * Math.sqrt(-Q) * Math.cos(theta / 3 + 240 * Math.PI / 180) - a1 / 3
    if (X1>0){
        solutions.push(X1)
    }
    if (X2>0){
        solutions.push(X2)
    }
    if (X3>0){
        solutions.push(X3)
    }
    return Math.min(solutions[0],solutions[1])


}

function maxClimbTOThrust(Ctc1, Ctc2, Ctc3, Hp) {
    let HpFeet = Hp * 3.28084
    return Ctc1 * (1 - HpFeet / Ctc2 + Ctc3 * HpFeet ** 2)
}

function correctMaxClimbTOThrust(Ctc1, Ctc2, Ctc3, Ctc4, Ctc5, Hp, deltaT) {
    let deltaTeff = deltaT - Ctc4
    return maxClimbTOThrust(Ctc1, Ctc2, Ctc3, Hp) * (1 - Ctc5 * deltaTeff)
}

function maxCruiseThrust(maxClimbThrust) {
    return .95 * maxClimbThrust
}


