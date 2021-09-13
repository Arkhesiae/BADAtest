import {createCharts} from "./charts.js"
import {knotToMs, msToKnot, PhysicalPlane} from "./formBADA.js"


const altitudeGraph = document.getElementById('altitude').getContext('2d');
const speedGraph = document.getElementById('speed').getContext('2d');
const forceGraph = document.getElementById('force').getContext('2d');
const ROCDGraph = document.getElementById('ROCD').getContext('2d');
const machGraph = document.getElementById('mach').getContext('2d');

let dataSets = {
    time: {
        data: [],
    },
    altitude: {
        data: [],
        graph: altitudeGraph,
    },
    mach: {
        data: {
            mach:[],
            buffet:[],
            maxMach:[],
        },
        graph: machGraph,
    },
    ROCD: {
        data: [],
        graph: ROCDGraph,
    },
    speed: {
        data: {
            CAS:[],
            TAS:[],
        },
        graph: speedGraph,
    },

    force: {
        data: {
            thrust:[],
            drag:[],
            maxThrust:[],
        },
        graph: forceGraph,
    },
}

let plane = new PhysicalPlane()
plane.setParameters(68000)
plane.setInitialState(3000, knotToMs(280), 0, 0)
plane.setLoi(280, 0.78)

let currentTime = 0
let initialTime = 0
let finalTime = 1500
let timeLoop
let increment = 1
for (let time = initialTime ; time<=finalTime ; time+=increment){
    currentTime++
    if (time<100){
        plane.monteeCASConstant()
    } else if (time<600){
        plane.palier()
    } else {
        plane.palier()
    }

    dataSets.time.data.push(currentTime)
    dataSets.altitude.data.push(Math.floor(plane.flightParams.Hp*3.28084))
    dataSets.speed.data.CAS.push(msToKnot(plane.flightParams.speed.CAS))
    dataSets.speed.data.TAS.push(msToKnot(plane.flightParams.speed.TAS))
    dataSets.ROCD.data.push(plane.flightParams.ROCD*196.85)
    dataSets.force.data.thrust.push(plane.force.thrust)
    dataSets.force.data.drag.push(plane.force.drag)
    dataSets.force.data.maxThrust.push(plane.maxThrust)
    dataSets.mach.data.buffet.push(plane.lowSpeedBuffetingLimit)
    dataSets.mach.data.mach.push(plane.flightParams.speed.Mach)
    dataSets.mach.data.maxMach.push(plane.maxMach)
}

let charts = createCharts(dataSets)
function addAllData(){
    charts[0].chart.data.labels.push(currentTime)
    charts.forEach((chart)=>{

        switch (chart.label){
            case "altitude":
                addData(chart.chart, currentTime, Math.floor(plane.flightParams.Hp*3.28084))
                break
            case "mach":
                addData(chart.chart, currentTime, {mach:plane.flightParams.speed.Mach, buffet :plane.lowSpeedBuffetingLimit, maxMach: plane.maxMach})
                break
            case "speed":
                addData(chart.chart, currentTime, {CAS:msToKnot(plane.flightParams.speed.CAS), TAS: msToKnot(plane.flightParams.speed.TAS)})
                break
            case "ROCD":
                addData(chart.chart, currentTime, Math.floor(plane.flightParams.ROCD*196.85))
                break
            case "force":
                addData(chart.chart, currentTime, {thrust: plane.force.thrust, drag : plane.force.drag, maxThrust: plane.maxThrust})
        }
    })
}

function timeLoopFunc(func){
    clearInterval(timeLoop)
    timeLoop = setInterval(()=>{
        addAllData()
        currentTime++
        func()
        // charts.forEach((chart)=>{
        //     chart.chart.update()
        // })
        charts.forEach((chart)=>{
            chart.chart.update()
        })
    },16)
}

function pressButton(button, func, input){
    button.addEventListener("click", function(event){
        let inputValue = input.value
        for (let time = 0 ; time<=inputValue ; time++){
            func()
            currentTime++
            addAllData()
        }
        timeLoopFunc(func)
        charts.forEach((chart)=>{
            chart.chart.update()
        })
    })
}

let climbButtonVitesse = document.getElementById("button-montee-vitesse")
let climbButtonAcc = document.getElementById("button-montee-acc")
let climbButtonDec = document.getElementById("button-montee-dec")
let climbButtonROCD = document.getElementById("button-montee-ROCD")
let descentButtonROCD = document.getElementById("button-descent-ROCD")
let descentButtonVitesse = document.getElementById("button-descent-vitesse")
let descentButtonAcc = document.getElementById("button-descent-acc")
let descentButtonDec = document.getElementById("button-descent-dec")
let climbInput = document.getElementById("climbInput")
let descentInput = document.getElementById("inputTempsDescent")

pressButton(climbButtonVitesse, plane.monteeCASConstant.bind(plane), climbInput)
pressButton(climbButtonAcc, plane.monteeAccel.bind(plane), climbInput)
pressButton(climbButtonDec, plane.monteeDecel.bind(plane), climbInput)
pressButton(descentButtonVitesse, plane.desCASConstant.bind(plane), descentInput)
pressButton(descentButtonAcc, plane.descentAccel.bind(plane), descentInput)
pressButton(descentButtonDec, plane.descentDecel.bind(plane), descentInput)

climbButtonROCD.addEventListener("click", function(event){
    let input = climbInput.value
    let targetROCD = document.getElementById("ROCDCLBtgt").value*0.00508
    for (let time = 0 ; time<=input ; time++){
        plane.climbAtROCD(targetROCD)
        currentTime++
        addAllData()
    }
    charts.forEach((chart)=>{
        chart.chart.update()
    })
})

descentButtonROCD.addEventListener("click", function(event){
    let input = document.getElementById("inputTempsDescent").value
    let targetROCD = document.getElementById("ROCDCLBtgt").value*0.00508
    for (let time = 0 ; time<=input ; time++){
        plane.descentAtROCD(targetROCD)
        currentTime++
        addAllData()
    }
    charts.forEach((chart)=>{
        chart.chart.update()
    })
})

function addData(chart, label, data) {
    if (chart.data.datasets.length>1){
        chart.data.datasets.forEach((dataset) => {
            dataset.data.push(data[dataset.label]);
        });
    }
    else{
        chart.data.datasets[0].data.push(data)
    }
}